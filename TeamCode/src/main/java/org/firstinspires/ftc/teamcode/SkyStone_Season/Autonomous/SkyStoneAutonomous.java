package org.firstinspires.ftc.teamcode.SkyStone_Season.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.SkyStone_Season.SkystoneRobotCfg;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import ftc.electronvolts.statemachine.BasicAbstractState;
import ftc.electronvolts.statemachine.EndCondition;
import ftc.electronvolts.statemachine.EndConditions;
import ftc.electronvolts.statemachine.State;
import ftc.electronvolts.statemachine.StateMachine;
import ftc.electronvolts.statemachine.StateMap;
import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.util.BasicResultReceiver;
import ftc.electronvolts.util.InputExtractor;
import ftc.electronvolts.util.ResultReceiver;
import ftc.electronvolts.util.TeamColor;
import ftc.electronvolts.util.Vector2D;
import ftc.electronvolts.util.files.Logger;
import ftc.electronvolts.util.files.OptionsFile;
import ftc.electronvolts.util.units.Angle;
import ftc.electronvolts.util.units.Distance;
import ftc.evlib.hardware.control.MecanumControl;
import ftc.evlib.hardware.control.RotationControls;
import ftc.evlib.hardware.control.TranslationControls;
import ftc.evlib.hardware.sensors.AnalogSensor;
import ftc.evlib.hardware.sensors.Gyro;
import ftc.evlib.opmodes.AbstractAutoOp;
import ftc.evlib.statemachine.EVStateMachineBuilder;
import ftc.evlib.util.EVConverters;
import ftc.evlib.util.FileUtil;

@Autonomous(name = "SkyStoneAuto")

public class SkyStoneAutonomous extends AbstractAutoOp<SkystoneRobotCfg> {
    private Gyro gyro;
    private MecanumControl mecanumControl;
    private OpenCvCamera camera;
    private BasicResultReceiver<Boolean> cameraInitRR = new BasicResultReceiver<>();
    private InputExtractor<StateName> s;
    private TeamColor teamColor = TeamColor.BLUE;
    private int minCycles = 10;
    private BasicResultReceiver<StateName> skystonePosStateRR = new BasicResultReceiver<>();
    private BasicResultReceiver<Boolean> canUpdateSRR = new BasicResultReceiver<>();
    private ProcessPipeline pipeline;
    private boolean doSkyStone = true;
    private Thread cameraInit;
    private ResultReceiver<Boolean> startRR = new BasicResultReceiver<>();

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
        pipeline = new ProcessPipeline(skystonePosStateRR, minCycles, teamColor, canUpdateSRR);
        mecanumControl = robotCfg.getMecanumControl();

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
                cameraInitRR.setValue(true);

            }
        };

        cameraInit = new Thread(r);
    }

    @Override
    protected void setup_act() {
        telemetry.addData("state", stateMachine.getCurrentStateName());
        telemetry.addData("Skystone position", skystonePosStateRR.isReady() ? skystonePosStateRR.getValue() : "not ready");
        telemetry.addData("forwardX", robotCfg.getPlusXDistanceSensor().cmUltrasonic());
        telemetry.addData("backwardX", robotCfg.getMinusXDistanceSensor().cmUltrasonic());
        telemetry.addData("forwardY", robotCfg.getPlusYDistanceSensor().getValue());
        robotCfg.getPlusYDistanceSensor().act();
        stateMachine.act();
    }

    @Override
    protected void go() {
        startRR.setValue(true);
        canUpdateSRR.setValue(false);
    }


    @Override
    protected void act() {
        telemetry.addData("gyro", robotCfg.getGyro().getHeading());
        telemetry.addData("state", stateMachine.getCurrentStateName());
        telemetry.addData("current thread", Thread.currentThread().getName());
        telemetry.addData("state for detetcting skystone", skystonePosStateRR.getValue());
        telemetry.addData("ratio of both stones", pipeline.getStoneRatioII().getValue());
        telemetry.addData("pods distance", robotCfg.getPlusYDistanceSensor().getValue());
    }





    @Override
    protected void end() {

    }

    @Override
    public StateMachine buildStates() {

        AnalogSensor podsSensor = new AnalogSensor() {
            @Override
            public Double getValue() {
                return robotCfg.getPlusYDistanceSensor().getValue();
            }
        };

        OptionsFile optionsFile = new OptionsFile(EVConverters.getInstance(), FileUtil.getOptionsFile(SkyStoneOptionsOp.FILENAME));
        teamColor = optionsFile.get(SkyStoneOptionsOp.Opts.TEAM_COLOR.s, SkyStoneOptionsOp.teamColorDefault);
        doSkyStone = optionsFile.get(SkyStoneOptionsOp.Opts.DO_SKYSTONE.s, SkyStoneOptionsOp.doSkyStoneDefault);
        pipeline = new ProcessPipeline(skystonePosStateRR, minCycles, teamColor, canUpdateSRR);
        EVStateMachineBuilder b = robotCfg.createEVStateMachineBuilder(S.INIT_GYRO, teamColor, Angle.fromDegrees(3));
        b.addCalibrateGyro(S.INIT_GYRO,S.POST_GYRO_WAIT);
        b.addWait(S.POST_GYRO_WAIT, S.INIT_CAMERA, 4000L);
        b.add(S.INIT_CAMERA, new BasicAbstractState() {
            @Override
            public void init() {
                cameraInit.start();
            }

            @Override
            public boolean isDone() {
                return cameraInitRR.isReady() && cameraInitRR.getValue();
            }

            @Override
            public StateName getNextStateName() {
                return S.POST_CAMERA_PAUSE;
            }
        });

        b.addWait(S.POST_CAMERA_PAUSE, S.WAIT_FOR_START, 1000L);
        b.addResultReceiverReady(S.WAIT_FOR_START, S.DRIVE_FORWARD, startRR);

        b.addDrive(S.DRIVE_FORWARD, S.DRIVE_TO_STONES, Distance.fromFeet(0.9), 0.8, 90,0);

        b.addDrive(S.DRIVE_TO_STONES, StateMap.of(
                S.STOP, EndConditions.timed(3000),
                S.WAIT_1, valueBetween(7, podsSensor, 9, 1)
        ), RotationControls.gyro(gyro, 1, Angle.fromDegrees(0), Angle.fromDegrees(2.5), 0.7),
                TranslationControls.sensor(podsSensor, 0.015, new Vector2D(0.5, Angle.fromDegrees(90)), 0.08, 9));
        b.addWait(S.WAIT_1, S.DECIDE_SKYSTONE, 500);
        b.add(S.DECIDE_SKYSTONE, getSkyStonePosition());
        b.addDrive(S.SKYSTONE_LEFT, S.GRABBLOCK, Distance.fromFeet(0.2), 0.5, 180, 0);
        b.addDrive(S.SKYSTONE_MIDDLE, S.GRABBLOCK, Distance.fromFeet(0.1), 0.5, 0, 0);
        b.addDrive(S.SKYSTONE_RIGHT, S.GRABBLOCK, Distance.fromFeet(0.3), 0.5, 0, 0);


        b.addStop(S.STOP1);


        if (doSkyStone) {

            b.add(S.STOP_CAMERA, createProcessState());
        b.addDrive(S.SKYSTONE_DRIVE_TO_LINE, S.PROCESS_SKYSTONE, Distance.fromFeet(.63), 0.25, 90, 0);
        b.add(S.PROCESS_SKYSTONE, getSkyStonePosition());

            if(teamColor == TeamColor.BLUE) {

                b.addDrive(S.SKYSTONE_LEFT, S.TURN_FOR_LEFT, Distance.fromFeet(2), .25, 180, 0);
                b.addGyroTurn(S.TURN_FOR_LEFT, S.DRIVE_LEFT, 45, Angle.fromDegrees(2), 0.3);
                b.add(S.DRIVE_LEFT, createCollectorDriveState(S.PICKUP_SKYSTONE_LEFT, 45, 45, 0.13, .65, .3));
                b.add(S.PICKUP_SKYSTONE_LEFT, createTimedFlywheelState(S.SKYSTONE_LEFT_READY_FOR_BRIDGE, 0.4, 750));
                b.add(S.SKYSTONE_LEFT_READY_FOR_BRIDGE, createCollectorDriveState(S.SKYSTONE_LEFT_DRIVE_TO_BRIDGE, -90, 90, .3, .62, .3));
                b.addDrive(S.SKYSTONE_LEFT_DRIVE_TO_BRIDGE, S.SKYSTONE_DRIVE_TO_BUILDING_SITE, Distance.fromFeet(.02), .3, 180, 90);


//        b.addDrive(S.DRIVE_LEFT, S.GRAB_BLOCK_ONE, Distance.fromFeet(0.1), 0.15, 113,0);
                b.addDrive(S.SKYSTONE_MIDDLE, S.TURN_FOR_MIDDLE, Distance.fromFeet(1.2), 0.25, 180, 0);
                b.addGyroTurn(S.TURN_FOR_MIDDLE, S.DRIVE_MIDDLE, 45, Angle.fromDegrees(2), 0.3);
                b.add(S.DRIVE_MIDDLE, createCollectorDriveState(S.PICKUP_SKYSTONE_MIDDLE, 45, 45, 0.13, .65, .3));
                b.add(S.PICKUP_SKYSTONE_MIDDLE, createTimedFlywheelState(S.SKYSTONE_MIDDLE_READY_FOR_BRIDGE, 0.4, 750));
                b.add(S.SKYSTONE_MIDDLE_READY_FOR_BRIDGE, createCollectorDriveState(S.SKYSTONE_MIDDLE_TO_BRIDGE, -90, 90, .3, .605, .3));
                b.addDrive(S.SKYSTONE_MIDDLE_TO_BRIDGE, S.SKYSTONE_DRIVE_TO_BUILDING_SITE, Distance.fromFeet(.3), 0.30, 180, 90);


                b.addDrive(S.SKYSTONE_RIGHT, S.TURN_FOR_RIGHT, Distance.fromFeet(0.7), 0.25, 180, 0);
                b.addGyroTurn(S.TURN_FOR_RIGHT, S.DRIVE_RIGHT, 45, Angle.fromDegrees(2), 0.3);
                b.add(S.DRIVE_RIGHT, createCollectorDriveState(S.PICKUP_SKYSTONE_RIGHT, 45, 45, 0.13, .65, .3));
                b.add(S.PICKUP_SKYSTONE_RIGHT, createTimedFlywheelState(S.SKYSTONE_RIGHT_READY_FOR_BRIDGE, 0.4, 750));
                b.add(S.SKYSTONE_RIGHT_READY_FOR_BRIDGE, createCollectorDriveState(S.SKYSTONE_RIGHT_TO_BRIDGE, -90, 90, .3, .65, .3));
                b.addDrive(S.SKYSTONE_RIGHT_TO_BRIDGE, S.SKYSTONE_DRIVE_TO_BUILDING_SITE, Distance.fromFeet(.45), 0.30, 180, 90);


                b.addDrive(S.SKYSTONE_DRIVE_TO_BUILDING_SITE, S.SKYSTONE_DRIVE_TO_FOUNDATION, Distance.fromFeet(1.65), 0.50, -180, 90);
                b.addDrive(S.SKYSTONE_DRIVE_TO_FOUNDATION, S.DROP_OFF_SKYSTONE, Distance.fromFeet(.95), 0.30, 90, 90);
                b.add(S.DROP_OFF_SKYSTONE, createTimedFlywheelState(S.FOUNDATIONMOVE_BACK_UP_TO_TURN, -1.0, 1250));
                b.add(S.FOUNDATIONMOVE_BACK_UP_TO_TURN, createCollectorDriveState(S.FOUNDATIONMOVE_TURN, -90, 90, 0.3, .5, -1.0));
                b.addGyroTurn(S.FOUNDATIONMOVE_TURN, S.FOUNDATIONMOVE_FORWARD, -90, Angle.fromDegrees(2), 0.3);
                b.addDrive(S.FOUNDATIONMOVE_FORWARD, S.LATCH_FOUNDATION, Distance.fromFeet(1.05), 0.12, 90, -90);
                b.addServo(S.LATCH_FOUNDATION, S.LATCH_FOUNDATION_RIGHT, SkystoneRobotCfg.SkystoneServoName.LEFT_FOUNDATION_MOVER_SERVO, SkystoneRobotCfg.LeftFoundationMoverServoPresets.DOWN, false);
                b.addServo(S.LATCH_FOUNDATION_RIGHT, S.PRE_DRAG_PAUSE, SkystoneRobotCfg.SkystoneServoName.RIGHT_FOUNDATION_MOVER_SERVO, SkystoneRobotCfg.RightFoundationMoverServoPresets.DOWN, true);
                b.addWait(S.PRE_DRAG_PAUSE, S.DRAG_FOUNDATION, 500);
                b.addDrive(S.DRAG_FOUNDATION, S.RELEASE_FOUNDATION, Distance.fromFeet(3.5), 0.25, 270, 270);
                b.addServo(S.RELEASE_FOUNDATION, S.RELEASE_FOUNDATION_RIGHT, SkystoneRobotCfg.SkystoneServoName.LEFT_FOUNDATION_MOVER_SERVO, SkystoneRobotCfg.LeftFoundationMoverServoPresets.UP, false);
                b.addServo(S.RELEASE_FOUNDATION_RIGHT, S.DRIVE_BACK_TO_BRIDGE, SkystoneRobotCfg.SkystoneServoName.RIGHT_FOUNDATION_MOVER_SERVO, SkystoneRobotCfg.RightFoundationMoverServoPresets.UP, true);
                b.addDrive(S.DRIVE_BACK_TO_BRIDGE, S.AVOID_ROBOT, Distance.fromFeet(0.9), 0.3, 0, 270);
                b.addDrive(S.AVOID_ROBOT, S.PARK, Distance.fromFeet(2), 0.3, 90, 270);
                b.addDrive(S.PARK, S.STOP, Distance.fromFeet(0.4), 0.3, 0, 270);

            } else {
                b.addDrive(S.RED_SKYSTONE_LEFT, S.RED_TURN_FOR_LEFT, Distance.fromFeet(0.8), .25, 0, 0);
                b.addGyroTurn(S.RED_TURN_FOR_LEFT, S.RED_READY_DRIVE_LEFT, 180, Angle.fromDegrees(2), 0.25);
                b.addDrive(S.RED_READY_DRIVE_LEFT, S.RED_DRIVE_LEFT, Distance.fromFeet(.4), 0.25, 90, 180);
                b.add(S.RED_DRIVE_LEFT, createCollectorDriveState(S.RED_PICKUP_SKYSTONE_LEFT, 180, 180, 0.13, .7, .25));
                b.add(S.RED_PICKUP_SKYSTONE_LEFT, createTimedFlywheelState(S.RED_SKYSTONE_LEFT_READY_FOR_BRIDGE, 0.4, 750));
                b.add(S.RED_SKYSTONE_LEFT_READY_FOR_BRIDGE, createCollectorDriveState(S.RED_SKYSTONE_LEFT_TURN_READY_FOR_BRIDGE, -90, 180, .3, 0.43, .3));
                b.addGyroTurn(S.RED_SKYSTONE_LEFT_TURN_READY_FOR_BRIDGE, S.RED_SKYSTONE_LEFT_DRIVE_TO_BRIDGE, 90, Angle.fromDegrees(2), 0.25);
                b.addDrive(S.RED_SKYSTONE_LEFT_DRIVE_TO_BRIDGE, S.RED_SKYSTONE_DRIVE_TO_BUILDING_SITE, Distance.fromFeet(.45), .3, 0, 90);


//        b.addDrive(S.DRIVE_LEFT, S.GRAB_BLOCK_ONE, Distance.fromFeet(0.1), 0.15, 113,0);
                b.addDrive(S.RED_SKYSTONE_MIDDLE, S.RED_TURN_FOR_MIDDLE, Distance.fromFeet(1.65), 0.25, 0, 0);
                b.addGyroTurn(S.RED_TURN_FOR_MIDDLE, S.RED_READY_DRIVE_MIDDLE, 180, Angle.fromDegrees(2), 0.25);
                b.addDrive(S.RED_READY_DRIVE_MIDDLE, S.RED_DRIVE_MIDDLE, Distance.fromFeet(.4), 0.25, 90, 180);
                b.add(S.RED_DRIVE_MIDDLE, createCollectorDriveState(S.RED_PICKUP_SKYSTONE_MIDDLE, 180, 180, 0.13, .7, .25));
                b.add(S.RED_PICKUP_SKYSTONE_MIDDLE, createTimedFlywheelState(S.RED_SKYSTONE_MIDDLE_READY_FOR_BRIDGE, 0.4, 750));
                b.add(S.RED_SKYSTONE_MIDDLE_READY_FOR_BRIDGE, createCollectorDriveState(S.RED_SKYSTONE_MIDDLE_TURN_READY_FOR_BRIDGE, -90, 180, .3, 0.43, .3));
                b.addGyroTurn(S.RED_SKYSTONE_MIDDLE_TURN_READY_FOR_BRIDGE, S.RED_SKYSTONE_MIDDLE_TO_BRIDGE, 90, Angle.fromDegrees(2), 0.25);
                b.addDrive(S.RED_SKYSTONE_MIDDLE_TO_BRIDGE, S.RED_SKYSTONE_DRIVE_TO_BUILDING_SITE, Distance.fromFeet(.3), 0.30, 0, 90);


                b.addDrive(S.RED_SKYSTONE_RIGHT, S.RED_TURN_FOR_RIGHT, Distance.fromFeet(2), 0.25, 0, 0);
                b.addGyroTurn(S.RED_TURN_FOR_RIGHT, S.RED_DRIVE_RIGHT, 135, Angle.fromDegrees(2), 0.23);
                b.addDrive(S.CORRECTING_DRIVE_RED, S.RED_DRIVE_RIGHT, Distance.fromFeet(0.1), 0.25, 100, 135);
                b.add(S.RED_DRIVE_RIGHT, createCollectorDriveState(S.PICKUP_SKYSTONE_RIGHT, 135, 127, 0.13, 0.8, .25));
                b.add(S.PICKUP_SKYSTONE_RIGHT, createTimedFlywheelState(S.RED_SKYSTONE_RIGHT_READY_FOR_BRIDGE, 0.4, 1250));
                b.add(S.RED_SKYSTONE_RIGHT_READY_FOR_BRIDGE, createCollectorDriveState(S.RED_SKYSTONE_RIGHT_TO_BRIDGE, 270, 90, .3, .65, .3));
                b.addDrive(S.RED_SKYSTONE_RIGHT_TO_BRIDGE, S.RED_SKYSTONE_DRIVE_TO_BUILDING_SITE, Distance.fromFeet(0.13), 0.30, 0, 90);


                b.addDrive(S.RED_SKYSTONE_DRIVE_TO_BUILDING_SITE, S.RED_SKYSTONE_DRIVE_TO_FOUNDATION, Distance.fromFeet(1.5), 0.50, 0, 90);
                b.addDrive(S.RED_SKYSTONE_DRIVE_TO_FOUNDATION, S.RED_DROP_OFF_SKYSTONE, Distance.fromFeet(1.05), 0.2, 90, 90);
                b.add(S.RED_DROP_OFF_SKYSTONE, createTimedFlywheelState(S.RED_FOUNDATIONMOVE_BACK_UP_TO_TURN, 0, 1250));
                b.add(S.RED_FOUNDATIONMOVE_BACK_UP_TO_TURN, createCollectorDriveState(S.RED_FOUNDATIONMOVE_TURN, 270, 90, 0.3, .5, 0));
                b.addGyroTurn(S.RED_FOUNDATIONMOVE_TURN, S.RED_FOUNDATIONMOVE_FORWARD, -90, Angle.fromDegrees(2), 0.3);
                b.addDrive(S.RED_FOUNDATIONMOVE_FORWARD, S.RED_LATCH_FOUNDATION, Distance.fromFeet(0.95), 0.12, 90, 270);
                b.addServo(S.RED_LATCH_FOUNDATION, S.RED_LATCH_FOUNDATION_RIGHT, SkystoneRobotCfg.SkystoneServoName.LEFT_FOUNDATION_MOVER_SERVO, SkystoneRobotCfg.LeftFoundationMoverServoPresets.DOWN, false);
                b.addServo(S.RED_LATCH_FOUNDATION_RIGHT, S.RED_PRE_DRAG_PAUSE, SkystoneRobotCfg.SkystoneServoName.RIGHT_FOUNDATION_MOVER_SERVO, SkystoneRobotCfg.RightFoundationMoverServoPresets.DOWN, true);
                b.addWait(S.RED_PRE_DRAG_PAUSE, S.RED_DRAG_FOUNDATION, 500);
                b.addDrive(S.RED_DRAG_FOUNDATION, S.RED_RELEASE_FOUNDATION, Distance.fromFeet(3.5), 0.25, 270, 270);
                b.addServo(S.RED_RELEASE_FOUNDATION, S.RED_RELEASE_FOUNDATION_RIGHT, SkystoneRobotCfg.SkystoneServoName.LEFT_FOUNDATION_MOVER_SERVO, SkystoneRobotCfg.LeftFoundationMoverServoPresets.UP, false);
                b.addServo(S.RED_RELEASE_FOUNDATION_RIGHT, S.RED_DRIVE_BACK_TO_BRIDGE, SkystoneRobotCfg.SkystoneServoName.RIGHT_FOUNDATION_MOVER_SERVO, SkystoneRobotCfg.RightFoundationMoverServoPresets.UP, true);
                b.addDrive(S.RED_DRIVE_BACK_TO_BRIDGE, S.RED_AVOID_ROBOT, Distance.fromFeet(0.73), 0.3, 180, 270);
                b.addDrive(S.RED_AVOID_ROBOT, S.RED_PARK, Distance.fromFeet(2), 0.3, 90, 270);
                b.addDrive(S.RED_PARK, S.STOP, Distance.fromFeet(0.4), 0.3, 180, 270);
            }
            }else{
             if(teamColor==TeamColor.RED) {
                 b.addDrive(S.STOP_CAMERA, S.FOUNDATION_DRIVE_2, Distance.fromFeet(2), 0.3, 180, 0);
                 b.addDrive(S.FOUNDATION_DRIVE_2, S.FOUNDATION_DRIVE_3, Distance.fromFeet(.3), 0.3, 90, 0);
                 b.addDrive(S.FOUNDATION_DRIVE_3, S.FOUNDATION_GRAB_1, Distance.fromFeet(.5), 0.3, 180, 0);
                 b.addServo(S.FOUNDATION_GRAB_1, S.FOUNDATION_GRAB_2, SkystoneRobotCfg.SkystoneServoName.LEFT_FOUNDATION_MOVER_SERVO, SkystoneRobotCfg.LeftFoundationMoverServoPresets.DOWN, false);
                 b.addServo(S.FOUNDATION_GRAB_2, S.FOUNDATION_WAIT, SkystoneRobotCfg.SkystoneServoName.RIGHT_FOUNDATION_MOVER_SERVO, SkystoneRobotCfg.RightFoundationMoverServoPresets.DOWN, true);
                 b.addWait(S.FOUNDATION_WAIT, S.FOUNDATION_DRIVE_4, 500);
                 b.addDrive(S.FOUNDATION_DRIVE_4, S.FOUNDATION_RELEASE_1, Distance.fromFeet(2.5), 0.3, 0, 0);
                 b.addServo(S.FOUNDATION_RELEASE_1, S.FOUNDATION_RELEASE_2, SkystoneRobotCfg.SkystoneServoName.LEFT_FOUNDATION_MOVER_SERVO, SkystoneRobotCfg.LeftFoundationMoverServoPresets.UP, false);
                 b.addServo(S.FOUNDATION_RELEASE_2, S.FOUNDATION_WAIT_2, SkystoneRobotCfg.SkystoneServoName.RIGHT_FOUNDATION_MOVER_SERVO, SkystoneRobotCfg.RightFoundationMoverServoPresets.UP, true);
                 b.addWait(S.FOUNDATION_WAIT_2, S.FOUNDATION_DRIVE_5, 500);
                 b.addDrive(S.FOUNDATION_DRIVE_5, S.STOP, Distance.fromFeet(1.0), 0.3, 270, 0);
             }else{
                 b.addDrive(S.STOP_CAMERA, S.BLUE_FOUNDATION_DRIVE_2, Distance.fromFeet(2), 0.3, 180, 0);
                 b.addDrive(S.BLUE_FOUNDATION_DRIVE_2, S.BLUE_FOUNDATION_DRIVE_3, Distance.fromFeet(.3), 0.3, 270, 0);
                 b.addDrive(S.BLUE_FOUNDATION_DRIVE_3, S.BLUE_FOUNDATION_WAIT, Distance.fromFeet(.5), 0.3, 180, 0);
                 b.addWait(S.BLUE_FOUNDATION_WAIT, S.BLUE_FOUNDATION_GRAB_1, 500);
                 b.addServo(S.BLUE_FOUNDATION_GRAB_1, S.BLUE_FOUNDATION_GRAB_2, SkystoneRobotCfg.SkystoneServoName.LEFT_FOUNDATION_MOVER_SERVO, SkystoneRobotCfg.LeftFoundationMoverServoPresets.DOWN, false);
                 b.addServo(S.BLUE_FOUNDATION_GRAB_2, S.BLUE_FOUNDATION_DRIVE_4, SkystoneRobotCfg.SkystoneServoName.RIGHT_FOUNDATION_MOVER_SERVO, SkystoneRobotCfg.RightFoundationMoverServoPresets.DOWN, true);
                 b.addDrive(S.BLUE_FOUNDATION_DRIVE_4, S.BLUE_FOUNDATION_RELEASE_1, Distance.fromFeet(2.5), 0.3, 0, 0);
                 b.addServo(S.BLUE_FOUNDATION_RELEASE_1, S.BLUE_FOUNDATION_RELEASE_2, SkystoneRobotCfg.SkystoneServoName.LEFT_FOUNDATION_MOVER_SERVO, SkystoneRobotCfg.LeftFoundationMoverServoPresets.UP, false);
                 b.addServo(S.BLUE_FOUNDATION_RELEASE_2, S.BLUE_FOUNDATION_WAIT_2, SkystoneRobotCfg.SkystoneServoName.RIGHT_FOUNDATION_MOVER_SERVO, SkystoneRobotCfg.RightFoundationMoverServoPresets.UP, true);
                 b.addWait(S.BLUE_FOUNDATION_WAIT_2, S.BLUE_FOUNDATION_DRIVE_5, 500);
                 b.addDrive(S.BLUE_FOUNDATION_DRIVE_5, S.STOP, Distance.fromFeet(1.0), 0.3, 90, 0);

             }
        }

        b.addStop(S.STOP);




      return b.build();
    }

    private EndCondition valueBetween(final int numRequiredInARow, final InputExtractor<Double> inputExtractor,
                                      double center, double delta) {

        final double min = center - delta;
        final double max = center + delta;

        return new EndCondition() {
            int numTimes;

            @Override
            public void init() {
                numTimes = 0;
            }

            @Override
            public boolean isDone() {
                double value = inputExtractor.getValue();
                if (min <= value && value <= max) {
                    numTimes++;
                } else {
                    numTimes = 0;
                }
                return numTimes >= numRequiredInARow;
            }
        };


    }




    private State createProcessState() {
        return new State() {
            @Override
            public StateName act() {
                if (skystonePosStateRR.isReady()) {
                    camera.closeCameraDevice();
                    return S.SKYSTONE_DRIVE_TO_LINE;
                }
                return null;
            }
        };
    }

    private State getSkyStonePosition() {
        return new State() {
            @Override
            public StateName act() {
                    return skystonePosStateRR.getValue();
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




    private State createCollectorDriveState(final StateName nextState, double direction,
                                            double orientation, double speed, double distance, final double collectorSpeed) {
        double maxAngSpeed = 0.5;
        final State s = ftc.evlib.statemachine.EVStates.mecanumDrive(nextState,
                Distance.fromFeet(distance),robotCfg.getMecanumControl(), robotCfg.getGyro(), speed,
                Angle.fromDegrees(direction), Angle.fromDegrees(orientation), Angle.fromDegrees(2), maxAngSpeed);

        return new BasicAbstractState() {
            @Override
            public void init() {
                s.act();
                robotCfg.getFlyWheels().setPower(collectorSpeed);
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
        DETECTION_1, GETRIGHTBLOCK, GETLEFTBLOCK, MIDDLE, GRABBLOCK, GOTOSIDE, GOBACKUP, UNLOAD, GOBACK, MOVETOBLOCKSAGAIN, GETLEFTBLOCKAGAIN, MIDDLEAGAIN, GETRIGHTBLOCKAGAIN, DRIVE_MIDDLE, DRIVE_RIGHT_BLUE, DRIVE_LEFT_BLUE, DRIVE_RIGHT_RED, DRIVE_LEFT_RED, GRAB_BLOCK_ONE, DRIVE_BACK, SKYSTONE_MIDDLE_TO_BRIDGE, SKYSTONE_CLOSE_TO_BRIDGE, SKYSTONE_FAR_TO_BRIDGE, DRIVE_TO_BRIDGE1, WAIT1, DRIVE_LEFT, DRIVE_RIGHT, INIT_GYRO, PICKUP_SKYSTONE1, PICKUP_SKYSTONE_LEFT, PICKUP_SYSTONE_RIGHT, PICKUP_SKYSTONE_RIGHT, SKYSTONE_LEFT_READY_FOR_BRIDGE, SKYSTONE_DRIVE_TO_FOUNDATION, SKYSTONE_DRIVE_TO_BUILDING_SITE, DROP_OFF_SKYSTONE, FOUNDATIONMOVE_BACK_UP_TO_TURN, FOUNDATIONMOVE_TURN, FOUNDATIONMOVE_FORWARD, SKYSTONE_DRIVE_TO_LINE, TURN_FOR_LEFT, STOP_CAMERA, TURN_FOR_MIDDLE, PICKUP_SKYSTONE_MIDDLE, SKYSTONE_MIDDLE_READY_FOR_BRIDGE, TURN_FOR_RIGHT, SKYSTONE_RIGHT_READY_FOR_BRIDGE, SKYSTONE_RIGHT_TO_BRIDGE, LET_GO_OF_FOUNDATION, MOVE_FOUNDATION_RIGHT, MOVE_FOUNDATION, LATCH_FOUNDATION_RIGHT, DRAG_FOUNDATION, LATCH_FOUNDATIOn, LATCH_FOUNDATION, PRE_DRAP_PAUSE, PRE_DRAG_PAUSE, RELEASE_FOUNDATION_RIGHT, RELEASE_FOUNDATION, DRIVE_BACK_TO_BRIDGE, PARK, AVOID_ROBOT, WAIT_BEFORE_DRIVE_TO_BUILDING_SITE, SKYSTONE_LEFT_DRIVE_TO_BRIDGE, RED_SKYSTONE_LEFT, RED_TURN_FOR_RIGHT, RED_DRIVE_RIGHT, RED_PICKUP_SKYSTONE_RIGHT, RED_SKYSTONE_LEFT_READY_FOR_BRIDGE, RED_SKYSTONE_LEFT_DRIVE_TO_BRIDGE, RED_SKYSTONE_DRIVE_TO_BUILDING_SITE, RED_SKYSTONE_DRIVE_TO_FOUNDATION, RED_DROP_OFF_SKYSTONE, RED_SKYSTONE_MIDDLE, RED_TURN_FOR_MIDDLE, RED_DRIVE_MIDDLE, RED_PICKUP_SKYSTONE_MIDDLE, RED_SKYSTONE_MIDDLE_READY_FOR_BRIDGE, RED_SKYSTONE_MIDDLE_TO_BRIDGE, RED_SKYSTONE_RIGHT_READY_FOR_BRIDGE, RED_SKYSTONE_RIGHT_TO_BRIDGE, RED_SKYSTONE_RIGHT, RED_PICKUP_SKYSTONE_LEFT, RED_DRIVE_LEFT, RED_TURN_FOR_LEFT, RED_FOUNDATIONMOVE_BACK_UP_TO_TURN, RED_FOUNDATIONMOVE_TURN, RED_FOUNDATIONMOVE_FORWARD, RED_LATCH_FOUNDATION, RED_LATCH_FOUNDATION_RIGHT, RED_PRE_DRAG_PAUSE, RED_DRAG_FOUNDATION, RED_RELEASE_FOUNDATION, RED_RELEASE_FOUNDATION_RIGHT, RED_DRIVE_BACK_TO_BRIDGE, RED_AVOID_ROBOT, RED_PARK, CORRECTING_DRIVE_RED, RED_READY_DRIVE_MIDDLE, RED_SKYSTONE_MIDDLE_TURN_READY_FOR_BRIDGE, RED_SKYSTONE_LEFT_TURN_READY_FOR_BRIDGE, RED_READY_DRIVE_LEFT, FOUNDATION_DRIVE_1, FOUNDATION_DRIVE_2, FOUNDATION_DRIVE_3, FOUNDATION_GRAB_1, FOUNDATION_GRAB_2, FOUNDATION_DRIVE_4, FOUNDATION_RELEASE_1, FOUNDATION_RELEASE_2, FOUNDATION_DRIVE_5, FOUNDATION_DRIVE_6, FOUNDATION_DRIVE_7, BLUE_FOUNDATION_DRIVE_2, BLUE_FOUNDATION_DRIVE_3, BLUE_FOUNDATION_GRAB_1, BLUE_FOUNDATION_GRAB_2, BLUE_FOUNDATION_DRIVE_4, BLUE_FOUNDATION_RELEASE_1, BLUE_FOUNDATION_RELEASE_2, BLUE_FOUNDATION_DRIVE_5, BLUE_FOUNDATION_DRIVE_6, BLUE_FOUNDATION_DRIVE_7, FOUNDATION_WAIT, BLUE_FOUNDATION_WAIT, FOUNDATION_WAIT_2, BLUE_FOUNDATION_WAIT_2, DRIVE_TO_STONES, STOP1, DRIVE_FORWARD, WAIT, WAIT_1, DRIVE_TO_LEFT_SKYSTONE, DECIDE_SKYSTONE, POST_GYRO_WAIT, INIT_CAMERA, POST_CAMERA_PAUSE, WAIT_FOR_START, DETECTION_2

    }
}
