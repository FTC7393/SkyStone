package org.firstinspires.ftc.teamcode.SkyStone_Season.Autonomous;
//comment

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import ftc.electronvolts.statemachine.BasicAbstractState;
import ftc.electronvolts.statemachine.State;
import ftc.electronvolts.statemachine.StateMachine;
import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.util.BasicResultReceiver;
import ftc.electronvolts.util.InputExtractor;
import ftc.electronvolts.util.TeamColor;
import ftc.electronvolts.util.files.Logger;
import ftc.electronvolts.util.units.Angle;
import ftc.electronvolts.util.units.Distance;
import ftc.evlib.hardware.control.MecanumControl;
import ftc.evlib.hardware.sensors.Gyro;
import ftc.evlib.opmodes.AbstractAutoOp;
import ftc.evlib.statemachine.EVStateMachineBuilder;

@Autonomous(name = "SkyStoneAuto JDV")

public class SkyStoneAutonomous extends AbstractAutoOp<SkystoneRobotCfg> {
    Gyro gyro;
    MecanumControl mecanumControl;
    OpenCvCamera phoneCam;
    private BasicResultReceiver<Boolean> phoneInitRR = new BasicResultReceiver<>();
    private BasicResultReceiver<StateName> foundSkyStoneRR = new BasicResultReceiver<>();
    private StateName postStoneDrive = null;
    InputExtractor<Double> avgColor;
    InputExtractor<Double> blue;
    ProcessPipeline processPipeline;

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
                phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
                phoneCam.openCameraDevice();
                int numContinuous = 5;
                int numSettle = 50;
                double minBlueValueForReg = 150.0;
                StateName [] nextStates = { S.A, S.B, S.C };
                int maxTries = 150;
                StateName defaultState = S.C;
                processPipeline = new ProcessPipeline(foundSkyStoneRR,numContinuous, numSettle, minBlueValueForReg, nextStates, maxTries, defaultState);
                avgColor = processPipeline.getAvgColorII();
                blue = processPipeline.getBlueDiffII();
                phoneCam.setPipeline(processPipeline);
                phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                phoneInitRR.setValue(true);
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
//        telemetry.addData("RunAvg: " , processPipeline.runAvgII.getValue());
        telemetry.addData("key: " , processPipeline.keyII.getValue());
        telemetry.addData("blue1: " , processPipeline.blue1II.getValue());
        telemetry.addData("blue2: " , processPipeline.blue2II.getValue());
        telemetry.addData("gyro", robotCfg.getGyro().getHeading());
        telemetry.addData("state", stateMachine.getCurrentStateName());
        telemetry.addData("thread", ProcessPipeline.threadName);
        telemetry.addData("current thread", Thread.currentThread().getName());
        telemetry.addData("average color", avgColor.getValue());
        telemetry.addData("blue", blue.getValue());
        if(blue.getValue() < 150) {
            telemetry.addData("found skystone!", blue);
        }

    }





    @Override
    protected void end() {

    }

    @Override
    public StateMachine buildStates() {
        TeamColor teamColor = TeamColor.RED;

        EVStateMachineBuilder b = robotCfg.createEVStateMachineBuilder(S.DRIVE_1, teamColor, Angle.fromDegrees(3));
        b.addDrive(S.DRIVE_1, S.PROCESS_SKYSTONE, Distance.fromFeet(0.1), 0.1, 90,0 );
        b.add(S.PROCESS_SKYSTONE, createProcessState());


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
                telemetry.addData("RR is ready", foundSkyStoneRR.isReady());
                if (foundSkyStoneRR.isReady()) {
                    telemetry.addData("next state", foundSkyStoneRR.getValue());
                }
                return false;
            }

            @Override
            public StateName getNextStateName() {
                return null;
            }
        };
    }


    private enum S implements StateName {

        DRIVE_1,
        PROCESS_SKYSTONE,
        DRIVE_STONE,
        STOP
        ,A,B,C; // bogus for example...

    }
}

