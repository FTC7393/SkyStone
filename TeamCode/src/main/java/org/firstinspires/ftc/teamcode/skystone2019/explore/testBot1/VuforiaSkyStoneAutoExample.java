package org.firstinspires.ftc.teamcode.skystone2019.explore.testBot1;

import com.google.common.collect.ImmutableList;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

import java.util.HashMap;
import java.util.Map;

import ftc.electronvolts.statemachine.BasicAbstractState;
import ftc.electronvolts.statemachine.EndCondition;
import ftc.electronvolts.statemachine.EndConditions;
import ftc.electronvolts.statemachine.State;
import ftc.electronvolts.statemachine.StateMachine;
import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.util.BasicResultReceiver;
import ftc.electronvolts.util.InputExtractor;
import ftc.electronvolts.util.ResultReceiver;
import ftc.electronvolts.util.TeamColor;
import ftc.electronvolts.util.Vector2D;
import ftc.electronvolts.util.files.Logger;
import ftc.electronvolts.util.units.Angle;
import ftc.electronvolts.util.units.Distance;
import ftc.evlib.hardware.control.RotationControl;
import ftc.evlib.hardware.control.RotationControls;
import ftc.evlib.opmodes.AbstractAutoOp;
import ftc.evlib.statemachine.EVStateMachineBuilder;
import ftc.evlib.statemachine.EVStates;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.mmPerInch;


/**
 * This 2019-2020 OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the SKYSTONE FTC field.
 * The code is structured as a LinearOpMode
 * <p>
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code then combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 * <p>
 * From the Audience perspective, the Red Alliance station is on the right and the
 * Blue Alliance Station is on the left.
 * <p>
 * Eight perimeter targets are distributed evenly around the four perimeter walls
 * Four Bridge targets are located on the bridge uprights.
 * Refer to the Field Setup manual for more specific location details
 * <p>
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  skystone/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 * <p>
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

@Autonomous(name = "TestBot Vuforia Auto")
public class VuforiaSkyStoneAutoExample extends AbstractAutoOp<TB2019RobotCfg> {

    private boolean vuforiaInitialized = false;
    private TeamColor teamColor;
    ResultReceiver<VuforiaManager> vfManagerResultRecvr = new BasicResultReceiver<>();

    @Override
    protected TB2019RobotCfg createRobotCfg() {
        return new TB2019RobotCfg(hardwareMap, CameraPlacement.createTestBotCP());
    }

    @Override
    public void setup() {
        super.setup();
        Runnable runnable = new Runnable() {
            @Override
            public void run() {
                VuforiaManager vfMgr = VuforiaManager.init(robotCfg.getCameraPlacement(), hardwareMap);
                vfMgr.activate();
                vfManagerResultRecvr.setValue(vfMgr);
            }
        };
        Thread t = new Thread(runnable);
        t.start();
    }


    @Override
    protected Logger createLogger() {
        ImmutableList.Builder<Logger.Column> cols = ImmutableList.builder();
        cols.addAll(robotCfg.getLoggerColumns());
        InputExtractor<?> stateMachLabelInptExt = new InputExtractor<Object>() {
            @Override
            public Object getValue() {
                return stateMachine.getCurrentStateName();
            }
        };
        cols.add(new Logger.Column("state", stateMachLabelInptExt));
        return new Logger("", "auto.csv",
                cols.build()
        );
    }


    @Override
    protected void setup_act() {

    }

    @Override
    protected void go() {

    }


    @Override
    protected void act() {
        telemetry.addData("gyro", robotCfg.getGyro().getHeading());
        telemetry.addData("state", stateMachine.getCurrentStateName());
    }


    @Override
    protected void end() {
//        mineralLogOutputter.close();
//        mineralLogOutputter = null;
    }


    private enum S implements StateName {
        WAIT_FOR_INIT, DRIVE1, TURN1, DRIVE2, MTT01_FIND_TARGET, MTT01_NO_TARGET_TURN, MTT01_FOUND_TARGET_S1, MTT01_FOUND_TARGET_S2, FOLLOW_TARGET, REACHED_TARGET, STOP;
    }

    @Override
    public StateMachine buildStates() {

        Angle tolerance = Angle.fromDegrees(3);
        StateName firstStateName = S.WAIT_FOR_INIT;
        EVStateMachineBuilder b = new EVStateMachineBuilder(firstStateName, teamColor, tolerance, robotCfg.getGyro(), robotCfg.getServos(), robotCfg.getMecanumControl());

        b.add(S.WAIT_FOR_INIT, new WaitForInitiState(S.DRIVE1, 5000L, vfManagerResultRecvr));
        b.addDrive(S.DRIVE1, S.TURN1, Distance.fromFeet(0.5), .9, 0, 0);
        b.addGyroTurn(S.TURN1, S.DRIVE2, -90, tolerance, 0.5);
        b.addDrive(S.DRIVE2, S.MTT01_FIND_TARGET, Distance.fromFeet(0.5), .9, -90, -90);
        BasicResultReceiver<VuforiaTrackableDefaultListener> target01_rr = new BasicResultReceiver();
        int tgtIdx = 9; // Blue Perim 1
        b.add(S.MTT01_FIND_TARGET, new FindTargetByIndex(tgtIdx, vfManagerResultRecvr, 15L, target01_rr,
                5000L, S.MTT01_NO_TARGET_TURN, S.MTT01_FOUND_TARGET_S1));
        // did not find the target - turn around 180 to +90
        b.addGyroTurn(S.MTT01_NO_TARGET_TURN, S.STOP, 90.0, Angle.fromDegrees(tolerance.degrees() * 2), 0.5);
        // did find target - so move to be a certain distance to it
        b.add(S.MTT01_FOUND_TARGET_S1, new WaitWithRunnable(S.MTT01_FOUND_TARGET_S2, 2000L, target01TelemActor(target01_rr)));
        b.addDrive(S.MTT01_FOUND_TARGET_S2, S.FOLLOW_TARGET, Distance.fromFeet(1.5), 0.3, -90, -90);
        Vector2D posn = new Vector2D(-42, 56);
        double posnGain = -0.02; // 0.0125;
        double toleranceInches = 2.0;
        b.add(S.FOLLOW_TARGET, createFollowTargetState(tgtIdx, posn, posnGain, toleranceInches, 22000L, S.REACHED_TARGET, S.STOP));
        b.addDrive(S.REACHED_TARGET, S.STOP, Distance.fromFeet(1), 0.4, -90, 90, 0.4);
        b.addStop(S.STOP);

        return b.build();
    }

    private Actor target01TelemActor(final BasicResultReceiver<VuforiaTrackableDefaultListener> target01_rr) {
        return new Actor() {
            OpenGLMatrix updatedTransform = null;

            @Override
            public void myAct() {
                VuforiaTrackableDefaultListener l = target01_rr.getValue();
                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                if (l.isVisible()) {
                    final OpenGLMatrix robotLocationTransform = l.getUpdatedRobotLocation();

                    if (robotLocationTransform != null) {
                        updatedTransform = robotLocationTransform;
                    }
                    if (updatedTransform != null) {
                        // express position (translation) of robot in inches.
                        VectorF translation = updatedTransform.getTranslation();
                        telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                                translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                        // express the rotation of the robot in degrees.
                        Orientation rotation = Orientation.getOrientation(updatedTransform, EXTRINSIC, XYZ, DEGREES);
                        telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                    } else {
                        telemetry.addData("Lost robot location!", "");
                    }
                } else {
                    telemetry.addData("No robot locn obtained", "");
                }
            }
        };
    }


//    @Override
//    public void runOpMode() {
//        while (!isStopRequested()) {
//
//            // check all the trackable targets to see which one (if any) is visible.
//            targetVisible = false;
//            for (VuforiaTrackable trackable : allTrackables) {
//                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
//                    telemetry.addData("Visible Target", trackable.getName());
//                    targetVisible = true;
//
//                    // getUpdatedRobotLocation() will return null if no new information is available since
//                    // the last time that call was made, or if the trackable is not currently visible.
//                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
//                    if (robotLocationTransform != null) {
//                        lastLocation = robotLocationTransform;
//                    }
//                    break;
//                }
//            }
//
//            // Provide feedback as to where the robot is located (if we know).
//            if (targetVisible) {
//                // express position (translation) of robot in inches.
//                VectorF translation = lastLocation.getTranslation();
//                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
//                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
//
//                // express the rotation of the robot in degrees.
//                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
//                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
//            } else {
//                telemetry.addData("Visible Target", "none");
//            }
//            telemetry.update();
//        }
//
//    }


    State createEndState(final VuforiaManager vm) {
        return new State() {
            @Override
            public StateName act() {
                // Disable Tracking when we are done;
                vm.getTargetsSkyStone().deactivate();
                return null;
            }
        };
    }



    State createFollowTargetState(final int targetIndex, final Vector2D desiredPosition, final double gain, final double toleranceInches, final long maxFollowMillis, final StateName foundTgtSt, final StateName lostTgtState) {

        RotationControl rotnCtrl = RotationControls.ZERO;
        final TargetFindingTranslationControl transCtrl = new TargetFindingTranslationControl(targetIndex,desiredPosition, vfManagerResultRecvr, gain, telemetry);

        InputExtractor<Boolean> distToTarget = new InputExtractor<Boolean>() {
            @Override
            public Boolean getValue() {
                return transCtrl.getDistanceToTarget() < toleranceInches;
            }
        };

        Map<StateName, EndCondition> endConds = new HashMap<>();
        endConds.put(lostTgtState, EndConditions.timed(maxFollowMillis));
        endConds.put(foundTgtSt, EndConditions.inputExtractor(distToTarget));
        return EVStates.mecanumDrive(endConds, robotCfg.getMecanumControl(), rotnCtrl, transCtrl);
    }
}

class WaitForInitiState extends BasicAbstractState {
    private final StateName nextState;
    private final long maxWait;
    private final ResultReceiver<VuforiaManager> vfMgrResRecvr;

    private long startTime;

    public WaitForInitiState(StateName nextState, long maxWait, ResultReceiver<VuforiaManager> vfMgrResRecvr) {
        this.nextState = nextState;
        this.maxWait = maxWait;
        this.vfMgrResRecvr = vfMgrResRecvr;
    }

    @Override
    public void init() {
        startTime = System.currentTimeMillis();
    }

    @Override
    public boolean isDone() {
        if (vfMgrResRecvr.isReady() || (System.currentTimeMillis() - startTime) > maxWait) {
            return true;
        }
        return false;
    }

    @Override
    public StateName getNextStateName() {
        return nextState;
    }
}


class FindTargetByIndex extends BasicAbstractState {

    private final int targetIndex;
    private final ResultReceiver<VuforiaManager> vfMgr_rr;
    private final long internalLoopDelay;
    private final ResultReceiver<VuforiaTrackableDefaultListener> rr;
    private final long maxWait;
    private final StateName stateIfNoneFound;
    private final StateName stateIfFound;

    private long startTime;

    public FindTargetByIndex(int targetIndex, ResultReceiver<VuforiaManager> vfMgr_rr, long internalLoopDelay, ResultReceiver<VuforiaTrackableDefaultListener> rr, long maxWait, StateName stateIfNoneFound, StateName stateIfFound) {
        this.targetIndex = targetIndex;
        this.vfMgr_rr = vfMgr_rr;
        this.internalLoopDelay = internalLoopDelay;
        this.rr = rr;
        this.maxWait = maxWait;
        this.stateIfNoneFound = stateIfNoneFound;
        this.stateIfFound = stateIfFound;
    }

    @Override
    public void init() {
        startTime = System.currentTimeMillis();
        final VuforiaTrackable desiredNavPic = vfMgr_rr.getValue().getTargetsSkyStone().get(targetIndex); // Rear 2

        Runnable r = new Runnable() {
            @Override
            public void run() {
                VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener) desiredNavPic.getListener();

                boolean found = false;
                while ((System.currentTimeMillis() - startTime) < maxWait) {
                    if (listener.isVisible()) {
                        rr.setValue(listener);
                        found = true;
                        break;
                    }
                    try {
                        Thread.sleep(internalLoopDelay);
                    } catch (InterruptedException e) {

                    }
                }
                if (!found) {
                    rr.setValue(null);
                }
            }
        };
        new Thread(r).start();
    }

    @Override
    public boolean isDone() {
        return rr.isReady();
    }

    @Override
    public StateName getNextStateName() {
        if (rr.getValue() == null) {
            return stateIfNoneFound;
        } else {
            return stateIfFound;
        }
    }
}


//class Q {
//    void R() {
//        if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
//            telemetry.addData("Visible Target", trackable.getName());
//            targetVisible = true;
//
//            // getUpdatedRobotLocation() will return null if no new information is available since
//            // the last time that call was made, or if the trackable is not currently visible.
//            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
//            if (robotLocationTransform != null) {
//                lastLocation = robotLocationTransform;
//            }
//            break;
//        }
//    }
//
//    // Provide feedback as to where the robot is located (if we know).
//            if (targetVisible) {
//        // express position (translation) of robot in inches.
//        VectorF translation = lastLocation.getTranslation();
//        telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
//                translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
//
//        // express the rotation of the robot in degrees.
//        Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
//        telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
//
//    }
//}


interface Actor {
    public void myAct();
}

class WaitWithRunnable extends BasicAbstractState {
    private final StateName nextStateName;
    private final long waitMillis;
    private final Actor actor;

    private long startTime;

    public WaitWithRunnable(StateName nextStateName, long waitMillis, Actor actor) {
        this.nextStateName = nextStateName;
        this.waitMillis = waitMillis;
        this.actor = actor;
    }

    @Override
    public void init() {
        startTime = System.currentTimeMillis();
    }

    @Override
    public boolean isDone() {
        actor.myAct();
        return (System.currentTimeMillis() - startTime) > waitMillis;
    }

    @Override
    public StateName getNextStateName() {
        return nextStateName;
    }
}


//class FollowTarget extends BasicAbstractState {
//    private final int targetIndex;
//    private final ResultReceiver<VuforiaManager> vfMgr_rr;
//    private final long maxFollowMillis;
//    private final StateName nextStateName;
//    private final long targetUpdateMillis = 20L;
//    private long startTime = -1;
//    private long lastVisibleTime = -1;
//    TargetTracker targetTracker = null;
//    ResultReceiver<Boolean> isDoneTrackingRR = null;
//    private long maxAllowedAgeMillis = 50L;
//    private double toleranceInches = 3;
//
//    public FollowTarget(int targetIndex, ResultReceiver<VuforiaManager> vfMgr_rr, long maxFollowMillis, StateName nextStateName) {
//        this.targetIndex = targetIndex;
//        this.vfMgr_rr = vfMgr_rr;
//        this.maxFollowMillis = maxFollowMillis;
//        this.nextStateName = nextStateName;
//    }
//
//    @Override
//    public void init() {
//        startTime = System.currentTimeMillis();
//        isDoneTrackingRR = new BasicResultReceiver<>();
//        // create new tracker
//        final VuforiaTrackable desiredNavPic = vfMgr_rr.getValue().getTargetsSkyStone().get(targetIndex); // Rear 2
//        VuforiaTrackableDefaultListener trListnr = (VuforiaTrackableDefaultListener) desiredNavPic.getListener();
//        Vector2D posn = new Vector2D(-44.4, 56);
//        targetTracker = new TargetTracker(trListnr, posn);
//        Runnable r = new Runnable() {
//            @Override
//            public void run() {
//                while(!isDoneTrackingRR.isReady()) {
//                    try {
//                        targetTracker.update();
//                        Thread.sleep(targetUpdateMillis);
//                    } catch (InterruptedException e) {
//                        // do nothing
//                    }
//                }
//            }
//        };
//        new Thread(r).start();
//
//
//    }
//
//    @Override
//    public boolean isDone() {
//        long now = System.currentTimeMillis();
//        if ((now - startTime) > maxFollowMillis) {
//            return true;
//        }
//        long obsAge = now - targetTracker.getLastTargetObsTime();
//        if (obsAge > maxAllowedAgeMillis) {
//            // pause motors
//
//        } else {
//            Vector2D d = targetTracker.getDisplacement();
//            if (d.getLength() < toleranceInches) {
//                return true;
//            }
//            // set motors to move along the displacement
//
//        }
//
//        return false;
//    }
//
//    @Override
//    public StateName getNextStateName() {
//        return nextStateName;
//    }
//}

class TargetTracker {
    private final VuforiaTrackableDefaultListener trListnr;
    private final Telemetry telem;
    private Vector2D currentPosition = null;
    private long lastTargetObsTime = -1;
    OpenGLMatrix updatedTransform = null;

    public TargetTracker(VuforiaTrackableDefaultListener trListnr, Telemetry telem) {
        this.trListnr = trListnr;
        this.telem = telem;
    }

    public Vector2D getPosition() {
//        if (currentPosition == null) {
//            // deal with this?
//        }
        return new Vector2D(currentPosition.getX(), currentPosition.getY());
    }

    public void update() {


        // getUpdatedRobotLocation() will return null if no new information is available since
        // the last time that call was made, or if the trackable is not currently visible.
        if (trListnr.isVisible()) {
            final OpenGLMatrix robotLocationTransform = trListnr.getUpdatedRobotLocation();

            if (robotLocationTransform != null) {
                updatedTransform = robotLocationTransform;
                lastTargetObsTime = System.currentTimeMillis();
            }
            if (updatedTransform != null) {
                // express position (translation) of robot in inches.
                VectorF translation = updatedTransform.getTranslation();
                currentPosition = new Vector2D(translation.get(0) / mmPerInch, translation.get(1) / mmPerInch);
                // express the rotation of the robot in degrees.
                telem.addData("Updated Xfrm?","YES");
            } else {
                telem.addData("Updated Xfrm?","NO");
            }
        }
        telem.addData("CURR POS  X,Y:", String.format("%4.1f   %4.1f",currentPosition.getX(), currentPosition.getY()));

    }

    public long getLastTargetObsTime() {
        return lastTargetObsTime;
    }
}