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
import ftc.electronvolts.util.units.Angle;
import ftc.electronvolts.util.units.Distance;
import ftc.evlib.hardware.control.MecanumControl;
import ftc.evlib.hardware.sensors.Gyro;
import ftc.evlib.opmodes.AbstractAutoOp;
import ftc.evlib.statemachine.EVStateMachineBuilder;

@Autonomous(name = "SkyStoneAuto")

public class SkyStoneAutonomous extends AbstractAutoOp<SkystoneRobotCfg> {
    Gyro gyro;
    MecanumControl mecanumControl;
    OpenCvCamera camera;
    private BasicResultReceiver<Boolean> rr = new BasicResultReceiver<>();
    InputExtractor<Double> avgColor;
    InputExtractor<Double> blue;
    int minCycles = 10;
    ProcessPipeline p = new ProcessPipeline(minCycles);

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
                avgColor = p.getAvgColorII();
                blue = p.getBlueDiffII();
                camera.setPipeline(p);
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
        telemetry.addData("average color", avgColor.getValue());
        telemetry.addData("blue one", p.blue);
        telemetry.addData("blue two", p.blue2);
        if (p.option == 0) {
            telemetry.addData("skystone middle", p.stoneratio);
        }
        if (p.option == 1) {
            telemetry.addData("skystone left", p.stoneratio);
        }
        if (p.option == 2) {
            telemetry.addData("skystone right", p.stoneratio);
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

    }
}
