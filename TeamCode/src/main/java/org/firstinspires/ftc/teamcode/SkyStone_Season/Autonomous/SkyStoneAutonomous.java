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
import ftc.electronvolts.util.ResultReceiver;
import ftc.electronvolts.util.TeamColor;
import ftc.electronvolts.util.files.Logger;
import ftc.electronvolts.util.files.OptionsFile;
import ftc.electronvolts.util.units.Angle;
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
    TeamColor tc = TeamColor.RED;
    InputExtractor<StateName> s;
    int minCycles = 10;
    private BasicResultReceiver<StateName> srr = new BasicResultReceiver<>();
    ProcessPipeline pipeline = new ProcessPipeline(srr, minCycles, tc);

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
        super.setup();
        Runnable r = new Runnable() {
            @Override
            public void run() {
                int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//                phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
                camera = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
                camera.openCameraDevice();
                s = pipeline.getStateNameII();
                pipeline.getStoneRatioII();
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

    }

    @Override
    protected void go() {

    }


    @Override
    protected void act() {
        telemetry.addData("gyro", robotCfg.getGyro().getHeading());
        telemetry.addData("state", stateMachine.getCurrentStateName());
        telemetry.addData("current thread", Thread.currentThread().getName());
        telemetry.addData("state for detetcting skystone", srr.getValue());
        telemetry.addData("ratio of both stones", pipeline.getStoneRatioII().getValue());
    }





    @Override
    protected void end() {

    }

    @Override
    public StateMachine buildStates() {
        TeamColor teamColor = TeamColor.RED;
        OptionsFile optionsFile = new OptionsFile(EVConverters.getInstance(), FileUtil.getOptionsFile(SkyStoneOptionsOp.FILENAME));

        ResultReceiver<Boolean> cont = new BasicResultReceiver<>();
        EVStateMachineBuilder b = robotCfg.createEVStateMachineBuilder(S.DRIVE_1, teamColor, Angle.fromDegrees(3));
//        b.addDrive(S.DRIVE_1, S.PROCESS_SKYSTONE, Distance.fromFeet(.6), 0.1, 270,0);
//        b.addDrive(S.DRIVE_1, S.PROCESS_SKYSTONE, Distance.fromFeet(1.1), 0.1, 0,0);
//        b.addBranch(S.DETECTION_1, S.GETRIGHTBLOCK, S.MIDDLE, S.GETLEFTBLOCK, cont);
//
//        //Move to different blocks depending on the webcam detection
//        b.addDrive(S.GETRIGHTBLOCK, S.GRABBLOCK, Distance.fromFeet(.1), 1, 180, 0);
//        b.addDrive(S.MIDDLE, S.GRABBLOCK, Distance.fromFeet(.2), 1, 0, 0);
//        b.addDrive(S.GETLEFTBLOCK, S.GRABBLOCK, Distance.fromFeet(.5), 1, 0, 0);
//
//        b.addServo(S.GRABBLOCK, S.GOBACKUP, SkystoneRobotCfg.SkystoneServoEnum.FINGERS_SERVO, SkystoneRobotCfg.FingersServoPresets.GRAB, true);
//        b.addDrive(S.GOBACKUP, S.GOTOSIDE, Distance.fromFeet(1), 1, 90, 0);
//        b.addDrive(S.GOTOSIDE, S.UNLOAD, Distance.fromFeet(4.5), 1, 180, 0);
//        //need to change this to servo for unload
//        b.addServo(S.GRABBLOCK, S.GOBACKUP, SkystoneRobotCfg.SkystoneServoEnum.FINGERS_SERVO, SkystoneRobotCfg.FingersServoPresets.RELEASE, true);
//
//        b.addDrive(S.GOBACK, S.MOVETOBLOCKSAGAIN, Distance.fromFeet(4.5), 0.1, 0, 0);
//        b.addDrive(S.MOVETOBLOCKSAGAIN, S.DETECTION_2 , Distance.fromFeet(1), 0.1, 270, 0);
//
//        // Picks up different block depending on original detection
//        b.addBranch(S.DETECTION_2, S.GETRIGHTBLOCKAGAIN, S.MIDDLEAGAIN, S.GETLEFTBLOCKAGAIN, cont);
//        b.addDrive(S.GETRIGHTBLOCKAGAIN, S.GRABBLOCK, Distance.fromFeet(.1), 1, 180, 0);
//        b.addDrive(S.MIDDLEAGAIN, S.GRABBLOCK, Distance.fromFeet(.2), 1, 0, 0);
//        b.addDrive(S.GETLEFTBLOCKAGAIN, S.GRABBLOCK, Distance.fromFeet(.5), 1, 0, 0);
//        b.addServo(S.GRABBLOCK, S.GOBACKUP, SkystoneRobotCfg.SkystoneServoEnum.FINGERS_SERVO, SkystoneRobotCfg.FingersServoPresets.GRAB, true);
//
//        b.addDrive(S.GOBACKUP, S.GOTOSIDE, Distance.fromFeet(1), 1, 90, 0);
//        b.addDrive(S.GOTOSIDE, S.UNLOAD, Distance.fromFeet(4.5), 1, 180, 0);
//        //need to change this to servo for unload
//        b.addServo(S.GRABBLOCK, S.GOBACKUP, SkystoneRobotCfg.SkystoneServoEnum.FINGERS_SERVO, SkystoneRobotCfg.FingersServoPresets.RELEASE, true);
//
//        // THIS WILL BE THE AREA OF CODE FOR MOVING THE FOUNDATION ONCE IT THE MECHANISM IS MADE
//        // ADD DISTANCES ARE NOT SET IN STONE AND ARE NEEDED TO BE CHANGED (ARBITRARY VALUES)
//
//        b.add(S.PROCESS_SKYSTONE, createProcessState());
        b.addStop(S.STOP);

      return b.build();
    }

    private State createProcessState() {
        return new BasicAbstractState() {
            @Override
            public void init() {

            }

            @Override
            public boolean isDone() {
                return false;
            }

            @Override
            public StateName getNextStateName() {
                return null;
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
        STOP,
        DETECTION_1, GETRIGHTBLOCK, GETLEFTBLOCK, MIDDLE, GRABBLOCK, GOTOSIDE, GOBACKUP, UNLOAD, GOBACK, MOVETOBLOCKSAGAIN, GETLEFTBLOCKAGAIN, MIDDLEAGAIN, GETRIGHTBLOCKAGAIN, DETECTION_2

    }
}
