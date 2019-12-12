package org.firstinspires.ftc.teamcode.SkyStone_Season.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.SkyStone_Season.SkystoneRobotCfg;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import ftc.electronvolts.statemachine.BasicAbstractState;
import ftc.electronvolts.statemachine.State;
import ftc.electronvolts.statemachine.StateMachine;
import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.util.BasicResultReceiver;
import ftc.electronvolts.util.InputExtractor;
import ftc.electronvolts.util.TeamColor;
import ftc.electronvolts.util.files.Logger;
import ftc.electronvolts.util.files.OptionsFile;
import ftc.electronvolts.util.units.Angle;
import ftc.electronvolts.util.units.Distance;
import ftc.evlib.hardware.control.MecanumControl;
import ftc.evlib.hardware.sensors.Gyro;
import ftc.evlib.opmodes.AbstractAutoOp;
import ftc.evlib.statemachine.EVStateMachineBuilder;
import ftc.evlib.util.EVConverters;
import ftc.evlib.util.FileUtil;

@Autonomous(name = "SkyStoneAuto")

public class SkyStoneAutonomous extends AbstractAutoOp<SkystoneRobotCfg> {
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

        gyro = robotCfg.getGyro();
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
    }





    @Override
    protected void end() {

    }

    @Override
    public StateMachine buildStates() {
        OptionsFile optionsFile = new OptionsFile(EVConverters.getInstance(), FileUtil.getOptionsFile(SkyStoneOptionsOp.FILENAME));
        pipeline = new ProcessPipeline(srr, minCycles, teamColor, canUpdateSRR);
        teamColor = optionsFile.get(SkyStoneOptionsOp.Opts.TEAM_COLOR.s, SkyStoneOptionsOp.teamColorDefault);
//        ResultReceiver<Boolean> cont = new BasicResultReceiver<>();
        EVStateMachineBuilder b = robotCfg.createEVStateMachineBuilder(S.INIT_GYRO, teamColor, Angle.fromDegrees(3));
        b.addCalibrateGyro(S.INIT_GYRO,S.PROCESS_SKYSTONE);
        b.add(S.PROCESS_SKYSTONE, createProcessState());
        b.addDrive(S.SKYSTONE_LEFT, S.DRIVE_LEFT, Distance.fromFeet(1.0), 0.30, 125, 45);
        b.add(S.DRIVE_LEFT, createPickupState(S.PICKUP_SKYSTONE_LEFT, 45, 45, 0.15, 0.4));
        b.add(S.PICKUP_SKYSTONE_LEFT, createTimedFlywheelState(S.STOP, 0.45,1500));

//        b.addDrive(S.DRIVE_LEFT, S.GRAB_BLOCK_ONE, Distance.fromFeet(0.1), 0.15, 113,0);
        b.addDrive(S.SKYSTONE_MIDDLE, S.PICKUP_SKYSTONE_RIGHT, Distance.fromFeet(0.68), 0.30, 98, 90);
        b.add(S.PICKUP_SKYSTONE_RIGHT, createTimedFlywheelState(S.STOP,0.45,1500));
        b.addDrive(S.SKYSTONE_RIGHT, S.DRIVE_RIGHT, Distance.fromFeet(0.76), 0.30, 86.3, 0);
//        b.add(S.PICKUP_SKYSTONE1, createPickupState(S.STOP, 90,0,0.4,0.25));
        b.addWait(S.WAIT1, S.DRIVE_BACK, 500);
        b.addDrive(S.DRIVE_BACK, S.STOP, Distance.fromFeet(0.35), 0.30, 270, 0);

        b.add(S.DRIVE_TO_BRIDGE1, createDriveToBridge1());

        b.addStop(S.STOP);

//        b.addDrive(S.DRIVE_LEFT_BLUE, S.GRABBLOCK, Distance.fromFeet(0.1), 0.4, 90, 90);
//        b.addServo(S.GRABBLOCK, S.GOBACKUP, SkystoneRobotCfg.SkystoneServoName.FINGERS_SERVO, SkystoneRobotCfg.FingersServoPresets.GRAB, true);
//        b.addDrive(S.DRIVE_1, S.PROCESS_SKYSTONE, Distance.fromFeet(1.1), 0.1, 0,0);
//        b.addBranch(S.DETECTION_1, S.GETRIGHTBLOCK, S.MIDDLE, S.GETLEFTBLOCK, cont);
//
//        //Move to different blocks depending on the webcam detection
//        b.addDrive(S.GETRIGHTBLOCK, S.GRABBLOCK, Distance.fromFeet(.1), 1, 180, 0);
//        b.addDrive(S.MIDDLE, S.GRABBLOCK, Distance.fromFeet(.2), 1, 0, 0);
//        b.addDrive(S.GETLEFTBLOCK, S.GRABBLOCK, Distance.fromFeet(.5), 1, 0, 0);
//
//        b.addServo(S.GRABBLOCK, S.GOBACKUP, SkystoneRobotCfg.SkystoneServoName.FINGERS_SERVO, SkystoneRobotCfg.FingersServoPresets.GRAB, true);
//        b.addDrive(S.GOBACKUP, S.GOTOSIDE, Distance.fromFeet(1), 1, 90, 0);
//        b.addDrive(S.GOTOSIDE, S.UNLOAD, Distance.fromFeet(4.5), 1, 180, 0);
//        //need to change this to servo for unload
//        b.addServo(S.GRABBLOCK, S.GOBACKUP, SkystoneRobotCfg.SkystoneServoName.FINGERS_SERVO, SkystoneRobotCfg.FingersServoPresets.RELEASE, true);
//
//        b.addDrive(S.GOBACK, S.MOVETOBLOCKSAGAIN, Distance.fromFeet(4.5), 0.1, 0, 0);
//        b.addDrive(S.MOVETOBLOCKSAGAIN, S.DETECTION_2 , Distance.fromFeet(1), 0.1, 270, 0);
//
//        // Picks up different block depending on original detection
//        b.addBranch(S.DETECTION_2, S.GETRIGHTBLOCKAGAIN, S.MIDDLEAGAIN, S.GETLEFTBLOCKAGAIN, cont);
//        b.addDrive(S.GETRIGHTBLOCKAGAIN, S.GRABBLOCK, Distance.fromFeet(.1), 1, 180, 0);
//        b.addDrive(S.MIDDLEAGAIN, S.GRABBLOCK, Distance.fromFeet(.2), 1, 0, 0);
//        b.addDrive(S.GETLEFTBLOCKAGAIN, S.GRABBLOCK, Distance.fromFeet(.5), 1, 0, 0);
//        b.addServo(S.GRABBLOCK, S.GOBACKUP, SkystoneRobotCfg.SkystoneServoName.FINGERS_SERVO, SkystoneRobotCfg.FingersServoPresets.GRAB, true);
//
//        b.addDrive(S.GOBACKUP, S.GOTOSIDE, Distance.fromFeet(1), 1, 90, 0);
//        b.addDrive(S.GOTOSIDE, S.UNLOAD, Distance.fromFeet(4.5), 1, 180, 0);
//        //need to change this to servo for unload
//        b.addServo(S.GRABBLOCK, S.GOBACKUP, SkystoneRobotCfg.SkystoneServoName.FINGERS_SERVO, SkystoneRobotCfg.FingersServoPresets.RELEASE, true);
//
//        // THIS WILL BE THE AREA OF CODE FOR MOVING THE FOUNDATION ONCE IT THE MECHANISM IS MADE
//        // ADD DISTANCES ARE NOT SET IN STONE AND ARE NEEDED TO BE CHANGED (ARBITRARY VALUES)
//


      return b.build();
    }

    private State createDriveToBridge1() {
        return new State() {
            @Override
            public StateName act() {
                if(srr.getValue() == SkyStoneAutonomous.S.SKYSTONE_LEFT) {
                    return SkyStoneAutonomous.S.SKYSTONE_CLOSE_TO_BRIDGE;
                } else if (srr.getValue() == SkyStoneAutonomous.S.SKYSTONE_MIDDLE) {
                    return SkyStoneAutonomous.S.SKYSTONE_MIDDLE_TO_BRIDGE;
                } else if (srr.getValue() == SkyStoneAutonomous.S.SKYSTONE_RIGHT){
                    return SkyStoneAutonomous.S.SKYSTONE_FAR_TO_BRIDGE;
                }
                else {
                    return SkyStoneAutonomous.S.SKYSTONE_FAR_TO_BRIDGE;
                }
            }
        };
    }


    private State createProcessState() {
        return new State() {
            @Override
            public StateName act() {
                if (srr.isReady()) {
                    camera.closeCameraDevice();
                    return srr.getValue();
                }
                return null;
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
                if((System.currentTimeMillis() - startTime) > durationMillis) {
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




    private State createPickupState(final StateName nextState, double direction,
                                    double orientation, double speed, double distance) {
        double maxAngSpeed = 0.5;
        final State s = ftc.evlib.statemachine.EVStates.mecanumDrive(nextState,
                Distance.fromFeet(distance),robotCfg.getMecanumControl(), robotCfg.getGyro(), speed,
                Angle.fromDegrees(direction), Angle.fromDegrees(orientation), Angle.fromDegrees(2), maxAngSpeed);

        return new BasicAbstractState() {
            @Override
            public void init() {
                s.act();
                robotCfg.getFlyWheels().setPower(0.5);
            }

            @Override
            public boolean isDone() {
                if(s.act() != null) {
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
        DETECTION_1, GETRIGHTBLOCK, GETLEFTBLOCK, MIDDLE, GRABBLOCK, GOTOSIDE, GOBACKUP, UNLOAD, GOBACK, MOVETOBLOCKSAGAIN, GETLEFTBLOCKAGAIN, MIDDLEAGAIN, GETRIGHTBLOCKAGAIN, DRIVE_MIDDLE, DRIVE_RIGHT_BLUE, DRIVE_LEFT_BLUE, DRIVE_RIGHT_RED, DRIVE_LEFT_RED, GRAB_BLOCK_ONE, DRIVE_BACK, SKYSTONE_MIDDLE_TO_BRIDGE, SKYSTONE_CLOSE_TO_BRIDGE, SKYSTONE_FAR_TO_BRIDGE, DRIVE_TO_BRIDGE1, WAIT1, DRIVE_LEFT, DRIVE_RIGHT, INIT_GYRO, PICKUP_SKYSTONE1, PICKUP_SKYSTONE_LEFT, PICKUP_SYSTONE_RIGHT, PICKUP_SKYSTONE_RIGHT, DETECTION_2

    }
}
