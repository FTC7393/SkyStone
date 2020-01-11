package org.firstinspires.ftc.teamcode.SkyStone_Season.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SkyStone_Season.SkystoneRobotCfg;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Map;

import ftc.electronvolts.statemachine.BasicAbstractState;
import ftc.electronvolts.statemachine.EndCondition;
import ftc.electronvolts.statemachine.EndConditions;
import ftc.electronvolts.statemachine.State;
import ftc.electronvolts.statemachine.StateMachine;
import ftc.electronvolts.statemachine.StateMap;
import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.util.BasicResultReceiver;
import ftc.electronvolts.util.InputExtractor;
import ftc.electronvolts.util.TeamColor;
import ftc.electronvolts.util.Vector2D;
import ftc.electronvolts.util.files.Logger;
import ftc.electronvolts.util.files.OptionsFile;
import ftc.electronvolts.util.units.Angle;
import ftc.electronvolts.util.units.Distance;
import ftc.evlib.hardware.config.RobotCfg;
import ftc.evlib.hardware.control.MecanumControl;
import ftc.evlib.hardware.control.RotationControls;
import ftc.evlib.hardware.control.TranslationControls;
import ftc.evlib.hardware.sensors.AnalogSensor;
import ftc.evlib.hardware.sensors.Gyro;
import ftc.evlib.opmodes.AbstractAutoOp;
import ftc.evlib.statemachine.EVStateMachineBuilder;
import ftc.evlib.util.EVConverters;
import ftc.evlib.util.FileUtil;
@Disabled
@Autonomous(name = "TestAutoWithDistanceSensor")

public class TestAutoWithDistanceSensor extends AbstractAutoOp<SkystoneRobotCfg> {
    Gyro gyro;
    MecanumControl mecanumControl;
    OpenCvCamera camera;
    private BasicResultReceiver<Boolean> rr = new BasicResultReceiver<>();
    InputExtractor<StateName> s;
    TeamColor teamColor = TeamColor.BLUE;
    int minCycles = 10;
    private BasicResultReceiver<StateName> srr = new BasicResultReceiver<>();
    private BasicResultReceiver<Boolean> canUpdateSRR = new BasicResultReceiver<>();
    private ProcessPipeline pipeline;
    private final double gyroGain = 0.6;
    private double maxAngularSpeed;


    @Override
    protected SkystoneRobotCfg createRobotCfg() {
        return new SkystoneRobotCfg(hardwareMap);
    }

    @Override
    protected Logger createLogger() {
        return null;
    }

    @Override
    public void setup() {
        gyro = robotCfg.getGyro();
        mecanumControl = robotCfg.getMecanumControl();
        pipeline = new ProcessPipeline(srr, 4, TeamColor.BLUE, canUpdateSRR);

        super.setup(); //Note: the superclass init method builds the state machine

        Runnable r = new Runnable() {
            @Override
            public void run() {
                int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

//                phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
                camera = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
                camera.openCameraDevice();
                s = pipeline.getStateNameII();
                camera.setPipeline(pipeline);
                camera.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);
                rr.setValue(true);

            }
        };

        Thread t = new Thread(r);
        t.start();
    }

    @Override
    protected void setup_act() {
        telemetry.addData("Skystone position", srr.isReady() ? srr.getValue() : "not ready");
    }

    @Override
    protected void go() {
        canUpdateSRR.setValue(false);
    }


    @Override
    protected void act() {
        telemetry.addData("gyro", robotCfg.getGyro().getHeading());
        telemetry.addData("state", stateMachine.getCurrentStateName());
        telemetry.addData("current thread", Thread.currentThread().getName());
        telemetry.addData("state for detetcting skystone", srr.getValue());
        telemetry.addData("ratio of both stones", pipeline.getStoneRatioII().getValue());
        telemetry.addData("state", stateMachine.getCurrentStateName());
        telemetry.addData("gyro angle", gyro.getHeading());
    }


    @Override
    protected void end() {

    }

    @Override
    public StateMachine buildStates() {
        EVStateMachineBuilder b = new EVStateMachineBuilder(S.INIT_GYRO, TeamColor.BLUE, Angle.fromDegrees(2.5),gyro, gyroGain, maxAngularSpeed, servos,mecanumControl);
        b.addCalibrateGyro(S.INIT_GYRO, S.DRIVE_1);

         AnalogSensor podsSensor = new AnalogSensor() {
            @Override
            public Double getValue() {
                return robotCfg.getPlusYDistanceSensor().getValue();
            }
        };

        if(teamColor == TeamColor.BLUE) {
            b.addDrive(S.DRIVE_1, S.DRIVE_2, Distance.fromFeet(0.5), 0.8, 90, 0, 0.5);
            b.addDrive(S.DRIVE_2, StateMap.of(
                    S.WAIT_1, EndConditions.timed(3000),
                    S.WAIT, EndConditions.valueCloseTo(podsSensor, 15, 1, true)
            ), RotationControls.gyro(gyro, gyroGain, Angle.fromDegrees(0), Angle.fromDegrees(2), 0.3),
                    TranslationControls.sensor(podsSensor, 0.02, new Vector2D(0.8, Angle.fromDegrees(90)),0.01, 15));
        }
        return b.build();
    }



    private State createProcessState() {
        return new State() {
            @Override
            public StateName act() {
                if (srr.isReady()) {
                    camera.closeCameraDevice();
                    return SkyStoneAutonomous.S.SKYSTONE_DRIVE_TO_LINE;
                }
                return null;
            }
        };
    }

    private State getSkyStonePosition() {
        return new State() {
            @Override
            public StateName act() {
                return srr.getValue();
            }

        };
    }

    private State createTimedFlywheelState(final StateName nextState, final double power, final long durationMillis) {

        return new BasicAbstractState() {
            long startTime;

            @Override
            public void init() {
                startTime = System.currentTimeMillis();
                robotCfg.getFlyWheels().setPower(power);
            }

            @Override
            public boolean isDone() {
                if ((System.currentTimeMillis() - startTime) > durationMillis) {
                    robotCfg.getFlyWheels().stop();
                    return true;
                }
                return false;
            }

            @Override
            public StateName getNextStateName() {
                return nextState;
            }
        };
    }


    private State createCollectorDriveState(final StateName nextState, double direction,
                                            double orientation, double speed, double distance, final double collectorSpeed) {
        double maxAngSpeed = 0.5;
        final State s = ftc.evlib.statemachine.EVStates.mecanumDrive(nextState,
                Distance.fromFeet(distance), robotCfg.getMecanumControl(), robotCfg.getGyro(),gyroGain, speed,
                Angle.fromDegrees(direction), Angle.fromDegrees(orientation), Angle.fromDegrees(2), maxAngSpeed);

        return new BasicAbstractState() {
            @Override
            public void init() {
                s.act();
                robotCfg.getFlyWheels().setPower(collectorSpeed);
            }

            @Override
            public boolean isDone() {
                if (s.act() != null) {
                    robotCfg.getFlyWheels().stop();
                    return true;
                }
                return false;
            }

            @Override
            public StateName getNextStateName() {
                return nextState;
            }
        };
    }


    public enum S implements StateName {

        DRIVE_1,
        PROCESS_SKYSTONE,
        DRIVE_STONE,
        SKYSTONE_MIDDLE,
        SKYSTONE_LEFT,
        SKYSTONE_RIGHT,
        DRIVE_2,
        STOP,
        DETECTION_1, GETRIGHTBLOCK, GETLEFTBLOCK, MIDDLE, GRABBLOCK, GOTOSIDE, GOBACKUP, UNLOAD, GOBACK, MOVETOBLOCKSAGAIN, GETLEFTBLOCKAGAIN, MIDDLEAGAIN, GETRIGHTBLOCKAGAIN, DRIVE_MIDDLE, DRIVE_RIGHT_BLUE, DRIVE_LEFT_BLUE, DRIVE_RIGHT_RED, DRIVE_LEFT_RED, GRAB_BLOCK_ONE, DRIVE_BACK, SKYSTONE_MIDDLE_TO_BRIDGE, SKYSTONE_CLOSE_TO_BRIDGE, SKYSTONE_FAR_TO_BRIDGE, DRIVE_TO_BRIDGE1, WAIT1, DRIVE_LEFT, DRIVE_RIGHT, INIT_GYRO, PICKUP_SKYSTONE1, PICKUP_SKYSTONE_LEFT, PICKUP_SYSTONE_RIGHT, PICKUP_SKYSTONE_RIGHT, SKYSTONE_LEFT_READY_FOR_BRIDGE, SKYSTONE_DRIVE_TO_FOUNDATION, SKYSTONE_DRIVE_TO_BUILDING_SITE, DROP_OFF_SKYSTONE, FOUNDATIONMOVE_BACK_UP_TO_TURN, FOUNDATIONMOVE_TURN, FOUNDATIONMOVE_FORWARD, SKYSTONE_DRIVE_TO_LINE, TURN_FOR_LEFT, STOP_CAMERA, TURN_FOR_MIDDLE, PICKUP_SKYSTONE_MIDDLE, SKYSTONE_MIDDLE_READY_FOR_BRIDGE, TURN_FOR_RIGHT, SKYSTONE_RIGHT_READY_FOR_BRIDGE, SKYSTONE_RIGHT_TO_BRIDGE, LET_GO_OF_FOUNDATION, MOVE_FOUNDATION_RIGHT, MOVE_FOUNDATION, LATCH_FOUNDATION_RIGHT, DRAG_FOUNDATION, LATCH_FOUNDATIOn, LATCH_FOUNDATION, PRE_DRAP_PAUSE, PRE_DRAG_PAUSE, RELEASE_FOUNDATION_RIGHT, RELEASE_FOUNDATION, DRIVE_BACK_TO_BRIDGE, PARK, AVOID_ROBOT, WAIT_BEFORE_DRIVE_TO_BUILDING_SITE, SKYSTONE_LEFT_DRIVE_TO_BRIDGE, RED_SKYSTONE_LEFT, RED_TURN_FOR_RIGHT, RED_DRIVE_RIGHT, RED_PICKUP_SKYSTONE_RIGHT, RED_SKYSTONE_LEFT_READY_FOR_BRIDGE, RED_SKYSTONE_LEFT_DRIVE_TO_BRIDGE, RED_SKYSTONE_DRIVE_TO_BUILDING_SITE, RED_SKYSTONE_DRIVE_TO_FOUNDATION, RED_DROP_OFF_SKYSTONE, RED_SKYSTONE_MIDDLE, RED_TURN_FOR_MIDDLE, RED_DRIVE_MIDDLE, RED_PICKUP_SKYSTONE_MIDDLE, RED_SKYSTONE_MIDDLE_READY_FOR_BRIDGE, RED_SKYSTONE_MIDDLE_TO_BRIDGE, RED_SKYSTONE_RIGHT_READY_FOR_BRIDGE, RED_SKYSTONE_RIGHT_TO_BRIDGE, RED_SKYSTONE_RIGHT, RED_PICKUP_SKYSTONE_LEFT, RED_DRIVE_LEFT, RED_TURN_FOR_LEFT, RED_FOUNDATIONMOVE_BACK_UP_TO_TURN, RED_FOUNDATIONMOVE_TURN, RED_FOUNDATIONMOVE_FORWARD, RED_LATCH_FOUNDATION, RED_LATCH_FOUNDATION_RIGHT, RED_PRE_DRAG_PAUSE, RED_DRAG_FOUNDATION, RED_RELEASE_FOUNDATION, RED_RELEASE_FOUNDATION_RIGHT, RED_DRIVE_BACK_TO_BRIDGE, RED_AVOID_ROBOT, RED_PARK, CORRECTING_DRIVE_RED, RED_READY_DRIVE_MIDDLE, RED_SKYSTONE_MIDDLE_TURN_READY_FOR_BRIDGE, RED_SKYSTONE_LEFT_TURN_READY_FOR_BRIDGE, RED_READY_DRIVE_LEFT, WAIT_1, WAIT, DETECTION_2

    }
}
