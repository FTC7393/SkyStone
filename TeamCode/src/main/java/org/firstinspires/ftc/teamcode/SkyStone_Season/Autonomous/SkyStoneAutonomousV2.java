package org.firstinspires.ftc.teamcode.SkyStone_Season.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SkyStone_Season.RobotV2.LiftArmStatesV2;
import org.firstinspires.ftc.teamcode.SkyStone_Season.RobotV2.LiftArmV2;
import org.firstinspires.ftc.teamcode.SkyStone_Season.RobotV2.SkystoneRobotCfgV2;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvInternalCamera;

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
import ftc.evlib.hardware.control.RotationControl;
import ftc.evlib.hardware.control.TranslationControl;
import ftc.evlib.hardware.control.TranslationControls;
import ftc.evlib.hardware.sensors.AnalogSensor;
import ftc.evlib.hardware.sensors.Gyro;
import ftc.evlib.hardware.sensors.SimpleEncoderSensor;
import ftc.evlib.opmodes.AbstractAutoOp;
import ftc.evlib.statemachine.EVStateMachineBuilder;
import ftc.evlib.util.EVConverters;
import ftc.evlib.util.FileUtil;
import ftc.evlib.util.ImmutableList;

@Autonomous(name = "SkyStoneAutoV2")

public class SkyStoneAutonomousV2 extends AbstractAutoOp<SkystoneRobotCfgV2> {
    private static final Angle SENSOR_ANGLE = Angle.fromDegrees(270);
    private Gyro gyro;
    private MecanumControl mecanumControl;
    private OpenCvCamera camera;
    private BasicResultReceiver<Boolean> cameraInitRR = new BasicResultReceiver<>();
    private InputExtractor<SkyStonePos> s;
    private TeamColor teamColor = TeamColor.BLUE;
    private int minCycles = 10;
    private BasicResultReceiver<SkyStonePos> skystonePosStateRR = new BasicResultReceiver<>();
    private BasicResultReceiver<Boolean> canUpdateSRR = new BasicResultReceiver<>();
    private ProcessPipeline pipeline;
    private boolean doSkyStone;
    private Thread cameraInit;
    private ResultReceiver<Boolean> startRR = new BasicResultReceiver<>();
    private double servoSpeed = 1;
    private OpenCvInternalCamera phoneCam;
    AnalogSensor plusYRaw = new AnalogSensor() {
        @Override
        public Double getValue() {
            return robotCfg.getPlusYDistanceSensor().getDistance(DistanceUnit.CM);
        }
    };
    AveragingAnalogSensor plusY = new AveragingAnalogSensor(plusYRaw);
    AnalogSensor minusXRaw = new AnalogSensor() {
        @Override
        public Double getValue() {
            return robotCfg.getMinusXDistanceSensor().getDistance(DistanceUnit.CM);
        }
    };
    AveragingAnalogSensor minusX = new AveragingAnalogSensor(minusXRaw);

    @Override
    protected SkystoneRobotCfgV2 createRobotCfg() {
        return new SkystoneRobotCfgV2(hardwareMap);
    }

    @Override
    protected Logger createLogger() {
       /** return new Logger("log", ".csv", ImmutableList.of(
                new Logger.Column("state", new InputExtractor<String>() {
                    @Override
                    public String getValue() {
                        return stateMachine.getCurrentStateName().name();
                    }
                }),
//                new Logger.Column("odometer wheel sensor", new InputExtractor<Double>() {
//                    @Override
//                    public Double getValue() {
//                        return robotCfg.getOdometryWheelSensor().getValue();
//                    }
//                }),
//                new Logger.Column("velocity", new InputExtractor<Double>() {
//                    @Override
//                    public Double getValue() {
//                        return TranslationControls.staticV;
//                    }
//                }),
                new Logger.Column("velocityX", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return TranslationControls.staticVX;
                    }
                }),
                new Logger.Column("velocityY", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return TranslationControls.staticVY;
                    }
                }),
//                new Logger.Column("lift arm position - left", new InputExtractor<Double>() {
//                    @Override
//                    public Double getValue() {
//                        return LiftArmV2.staticLiftLeft;
//
//                    }
//                }),
//                new Logger.Column("lift arm position - right", new InputExtractor<Double>() {
//                    @Override
//                    public Double getValue() {
//                        return LiftArmV2.staticLiftRight;
//                    }
//                }),
//                new Logger.Column("lift command linear slide - left", new InputExtractor<Double>() {
//                    @Override
//                    public Double getValue() {
//                        return robotCfg.getLiftArmV2().getVerticalSlideLeft().getExtensionSetPoint();
//                    }
//                }),
//                new Logger.Column("lift command linear slide - right", new InputExtractor<Double>() {
//                    @Override
//                    public Double getValue() {
//                        return robotCfg.getLiftArmV2().getVerticalSlideRight().getExtensionSetPoint();
//                    }
//                }),
//                new Logger.Column("horizontal limit switch", new InputExtractor<Boolean>() {
//                    @Override
//                    public Boolean getValue() {
//                        return robotCfg.getLiftArmV2().getLowerLimitHorizontal().getValue();
//                    }
//                }),
//                new Logger.Column("extension command", new InputExtractor<Double>() {
//                    @Override
//                    public Double getValue() {
//                        return robotCfg.getLiftArmV2().getExtensionCommand();
//                    }
//                }),
//                new Logger.Column("horizontal slide encoder", new InputExtractor<Double>() {
//                    @Override
//                    public Double getValue() {
//                        return robotCfg.getLiftArmV2().getHorizontalEncoder();
//                    }
//                }),
                new Logger.Column("plusY", plusY),
                new Logger.Column("minusX", minusX)
//                new Logger.Column("gyro", new InputExtractor<Double>() {
//                    @Override
//                    public Double getValue() {
//                        return robotCfg.getGyro().getHeading();
//                    }
//                })

        ));
*/
       return null;
    }

    @Override
    public void setup() {
        gyro = robotCfg.getGyro();
        mecanumControl = robotCfg.getMecanumControl();

        super.setup(); //Note: the superclass init method builds the state machine
        Runnable r = new Runnable() {
            @Override
            public void run() {
                int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
                phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
                //camera = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
                long cameraPause = 200L;
                sleep(cameraPause);
                phoneCam.openCameraDevice();
                sleep(cameraPause);
                s = pipeline.getStateNameII();
                phoneCam.setPipeline(pipeline);
                sleep(cameraPause);
                phoneCam.startStreaming(640, 480);
                sleep(cameraPause);
                cameraInitRR.setValue(true);

            }
        };

        cameraInit = new Thread(r);

        robotCfg.getOdometryServo().goToPreset(SkystoneRobotCfgV2.OdometryServoPresets.DOWN);
        robotCfg.getCapstoneServo().goToPreset(SkystoneRobotCfgV2.CapstonePlacementPresets.CLOSE);

    }

    private void sleep(long sleepTimeMillis) {
        try {
            Thread.sleep(sleepTimeMillis);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    @Override
    protected void setup_act() {
        telemetry.addData("state", stateMachine.getCurrentStateName());
        telemetry.addData("Skystone position", skystonePosStateRR.isReady() ? skystonePosStateRR.getValue() : "not ready");
        telemetry.addData("minusXDistance",robotCfg.getMinusXDistanceSensor().getDistance(DistanceUnit.CM));
        telemetry.addData("plusX distance sensor", robotCfg.getPlusXDistanceSensor().getDistance(DistanceUnit.CM));
        telemetry.addData("left limit switch", robotCfg.getLiftArmV2().getLowerLimitVerticalLeft().getValue());
        telemetry.addData("right limit switch", robotCfg.getLiftArmV2().getLowerLimitVerticalRight().getValue());
        OptionsFile optionsFile = new OptionsFile(EVConverters.getInstance(), FileUtil.getOptionsFile(SkyStoneOptionsOp.FILENAME));
        teamColor = optionsFile.get(SkyStoneOptionsOp.Opts.TEAM_COLOR.s, SkyStoneOptionsOp.teamColorDefault);
        telemetry.addData("team color", teamColor);
        stateMachine.act();
        servos.act();
    }

    @Override
    protected void go() {
        startRR.setValue(true);
    }


    @Override
    protected void act() {
//        telemetry.addData("gyro", robotCfg.getGyro().getHeading());
//        telemetry.addData("state", stateMachine.getCurrentStateName());
//        telemetry.addData("current thread", Thread.currentThread().getName());
//        telemetry.addData("state for detetcting skystone", skystonePosStateRR.getValue());
//        telemetry.addData("ratio of both stones", pipeline.getStoneRatioII().getValue());
//        telemetry.addData("odometer Encoder", robotCfg.getOdometryWheelSensor().getValue());
//        telemetry.addData("plusY distance sensor", robotCfg.getPlusYDistanceSensor().getDistance(DistanceUnit.CM));
//        telemetry.addData("plusX distance sensor", robotCfg.getPlusXDistanceSensor().getDistance(DistanceUnit.CM));
//        telemetry.addData("minusX distance sensor", robotCfg.getMinusXDistanceSensor().getDistance(DistanceUnit.CM));
    }





    @Override
    protected void end() {

    }

    @Override
    public StateMachine buildStates() {


        OptionsFile optionsFile = new OptionsFile(EVConverters.getInstance(), FileUtil.getOptionsFile(SkyStoneOptionsOp.FILENAME));
        RC rc = new RC(0.7, Angle.fromDegrees(2), 0.7, gyro);
        final SimpleEncoderSensor odSensor = robotCfg.getOdometryWheelSensor();
        teamColor = optionsFile.get(SkyStoneOptionsOp.Opts.TEAM_COLOR.s, SkyStoneOptionsOp.teamColorDefault);
        doSkyStone = optionsFile.get(SkyStoneOptionsOp.Opts.DO_SKYSTONE.s, SkyStoneOptionsOp.doSkyStoneDefault);
        pipeline = new ProcessPipeline(skystonePosStateRR, minCycles, teamColor, canUpdateSRR);
        Angle tolerance = Angle.fromDegrees(2.5);
        EVStateMachineBuilder b = new EVStateMachineBuilder(S.INIT_GYRO, teamColor, tolerance,
                gyro, 0.6, 0.7, robotCfg.getServos(), mecanumControl);
        b.addCalibrateGyro(S.INIT_GYRO,S.POST_GYRO_WAIT);
        b.addWait(S.POST_GYRO_WAIT, S.INIT_CAMERA, 2100);
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
        b.addWait(S.POST_CAMERA_PAUSE, S.WAIT_FOR_START, 500);
        b.addResultReceiverReady(S.WAIT_FOR_START, S.WAIT_FOR_SKYSTONE, startRR);
        b.addResultReceiverReady(S.WAIT_FOR_SKYSTONE, S.STOP_SKYSTONE_SEARCH, skystonePosStateRR);
        b.add(S.STOP_SKYSTONE_SEARCH, new State() {
            @Override
            public StateName act() {
                canUpdateSRR.setValue(false);
                return S.INIT_LIFT_ARM;
            }
        });

        b.add(S.INIT_LIFT_ARM, LiftArmStatesV2.liftMove(S.INIT_EXT_ARM, robotCfg.getLiftArmV2(), -10, false));
        b.add(S.INIT_EXT_ARM, LiftArmStatesV2.horizontalExtension(S.DECIDE_SKYSTONE_POSITION, robotCfg.getLiftArmV2(), -20, false));
//      b.addDrive(S.DRIVE_WITH_ODOMETRY, S.STOP, Distance.fromFeet(1), 0.8, Angle.fromDegrees(-90), Angle.fromDegrees(0));
        b.add(S.DECIDE_SKYSTONE_POSITION, getSkyStonePosition());



        addOdomDrive(null, S.SKYSTONE_RIGHT, b,rc, S.STOP1, 5000, S.MOVE_ARM_UP, 26.0,
                Angle.fromDegrees(292), 1, Angle.fromDegrees(0),
                200, 400, 3);
        addOdomDrive(null, S.SKYSTONE_LEFT, b, rc, S.STOP1, 5000, S.MOVE_ARM_UP, 29, Angle.fromDegrees(305),
                1, Angle.fromDegrees(0), 200, 400, 3);
        addOdomDrive(null, S.SKYSTONE_MIDDLE, b, rc, S.STOP1, 5000, S.MOVE_ARM_UP, 28, Angle.fromDegrees(300),
                1, Angle.fromDegrees(0), 200, 400, 3);

        b.add(S.MOVE_ARM_UP, LiftArmStatesV2.liftMove(S.TURN_1, robotCfg.getLiftArmV2(), 600, false));
        b.addGyroTurn(S.TURN_1, S.START_COLLECTOR, -45, Angle.fromDegrees(2));
        b.addMotorOn(S.START_COLLECTOR,S.ODOMETRY_RESET, robotCfg.getBlockCollector().getCollectorMotor(), -1);
        addOdomDrive(S.ODOMETRY_RESET, S.DRIVE_WITH_ODOMETRY_2,b,rc, S.STOP1, 5000, S.ODOMETRY_RESET_2,
                10, Angle.fromDegrees(-135), 0.75, Angle.fromDegrees(-45), 150, 300, 3);
        addOdomDrive(S.ODOMETRY_RESET_2, S.DRIVE_WITH_ODOMETRY_3, b,rc,S.STOP1, 5000, S.START_FINGER_RIGHT, -15,
                Angle.fromDegrees(-135), 0.75, Angle.fromDegrees(-45), 150, 300, 3);
        b.addServo(S.START_FINGER_RIGHT, S.START_FINGER_LEFT, robotCfg.getFingerRight().getName(), SkystoneRobotCfgV2.FingerRightServoPresets.FORWARD, false);
        b.addServo(S.START_FINGER_LEFT, S.MOVE_ARM_DOWN, robotCfg.getFingerLeft().getName(), SkystoneRobotCfgV2.FingerLeftServoPresets.FORWARD, false);
        b.add(S.MOVE_ARM_DOWN, LiftArmStatesV2.liftMove(S.STOP_COLLECTOR, robotCfg.getLiftArmV2(), 0, false));
        b.addMotorOff(S.STOP_COLLECTOR,S.TURN_2, robotCfg.getBlockCollector().getCollectorMotor());
        b.addGyroTurn(S.TURN_2, S.STOP_RIGHT_FINGER, -90, Angle.fromDegrees(1));
        b.addServo(S.STOP_RIGHT_FINGER, S.STOP_FINGER_LEFT, robotCfg.getFingerRight().getName(), SkystoneRobotCfgV2.FingerRightServoPresets.STOP, false);
        b.addServo(S.STOP_FINGER_LEFT, S.CHOOSE_DRIVE_DISTANCE, robotCfg.getFingerLeft().getName(), SkystoneRobotCfgV2.FingerLeftServoPresets.STOP, false);
        b.add(S.CHOOSE_DRIVE_DISTANCE, getDriveState());
        addOdomDrive(S.SKYSTONE_LEFT_DRIVE_TO_BRIDGE, S.DRIVE_WITH_ODOMETRY_8, b,rc,S.STOP1, 5000, S.TURN_3, -59,
                Angle.fromDegrees(0), 1, Angle.fromDegrees(-90), 200, 400, 3);
        addOdomDrive(S.SKYSTONE_MIDDLE_TO_BRIDGE, S.DRIVE_WITH_ODOMETRY_9, b,rc,S.STOP1, 5000, S.TURN_3, -67,
                Angle.fromDegrees(0), 1, Angle.fromDegrees(-90), 200, 400, 3);
        addOdomDrive(S.SKYSTONE_RIGHT_TO_BRIDGE, S.DRIVE_WITH_ODOMETRY_10, b,rc,S.STOP1, 5000, S.TURN_3, -75,
                Angle.fromDegrees(0), 1, Angle.fromDegrees(-90), 200, 400, 3);
        b.addDrive(S.DRIVE_DISTANCE_1, StateMap.of(
                S.STOP1, EndConditions.timed(6000),
                S.TURN_3, valueBetween(4, plusY, 30, 5)),rc.gyro(-90),
                TranslationControls.sensor2(plusY, 0.07, Angle.fromDegrees(180), new Vector2D(0.75, Angle.fromDegrees(180)), 0.014, 30, 3)
        );
        b.addGyroTurn(S.TURN_3, S.FOUNDATION_MOVER_READY_RIGHT, 180, Angle.fromDegrees(2));
        b.addServo(S.FOUNDATION_MOVER_READY_RIGHT, S.FOUNDATION_MOVER_READY_LEFT, robotCfg.getRightFoundationMover().getName(), SkystoneRobotCfgV2.RightFoundationMoverServoPresets.READY, false);
        b.addServo(S.FOUNDATION_MOVER_READY_LEFT, S.DRIVE_DISTANCE_2, robotCfg.getLeftFoundationMover().getName(), SkystoneRobotCfgV2.LeftFoundationMoverServoPresets.READY, false);

        b.addDrive(S.DRIVE_DISTANCE_2, StateMap.of(
                S.MOVE_ARM_UP_FOUNDATION, EndConditions.timed(6000),
            S.MOVE_ARM_UP_FOUNDATION, valueBetween(4, plusY, 3, 2)),rc.gyro(180),
                TranslationControls.sensor2(plusY, 0.07, Angle.fromDegrees(90), new Vector2D(0.75, Angle.fromDegrees(90)), 0.014, 3, 1)
        );
        b.add(S.MOVE_ARM_UP_FOUNDATION, LiftArmStatesV2.liftMove(S.EXTEND_SKYSTONE, robotCfg.getLiftArmV2(), 800, true));
        b.add(S.EXTEND_SKYSTONE, LiftArmStatesV2.horizontalExtension(S.DROP_SKYSTONE, robotCfg.getLiftArmV2(), 1800, true));
        b.addServo(S.DROP_SKYSTONE, S.DROP_SKYSTONE_WAIT, robotCfg.getGripper().getName(), SkystoneRobotCfgV2.GripperServoPresets.RELEASE,  true);
        b.addWait(S.DROP_SKYSTONE_WAIT, S.GRIPPER_READY_RETRACT, 700 );
        b.addServo(S.GRIPPER_READY_RETRACT, S.RETRACT_ARM, robotCfg.getGripper().getName(), SkystoneRobotCfgV2.GripperServoPresets.GRAB, false);
        b.add(S.RETRACT_ARM, LiftArmStatesV2.horizontalExtension(S.DRIVE_DISTANCE_3, robotCfg.getLiftArmV2(), 0, false));
        b.addDrive(S.DRIVE_DISTANCE_3, StateMap.of(
                S.STOP1, EndConditions.timed(6000),
                S.FOUNDATION_MOVER_DOWN_RIGHT, valueBetween(4, plusY, 6, 2)),rc.gyro(180),
                TranslationControls.sensor2(plusY, 0.07, Angle.fromDegrees(90), new Vector2D(0.75, Angle.fromDegrees(90)), 0.014, 6, 1)
        );

        b.addServo(S.FOUNDATION_MOVER_DOWN_RIGHT, S.FOUNDATION_MOVER_DOWN_LEFT, robotCfg.getRightFoundationMover().getName(), SkystoneRobotCfgV2.RightFoundationMoverServoPresets.DOWN, false);
        b.addServo(S.FOUNDATION_MOVER_DOWN_LEFT, S.MOVE_ARM_DOWN_2, robotCfg.getLeftFoundationMover().getName(), SkystoneRobotCfgV2.LeftFoundationMoverServoPresets.DOWN, false);
        b.add(S.MOVE_ARM_DOWN_2, LiftArmStatesV2.liftMove(S.ODOMETRY_RESET_3, robotCfg.getLiftArmV2(), 0, false));
        addOdomDrive(S.ODOMETRY_RESET_3, S.DRIVE_WITH_ODOMETRY_4, b, rc, S.STOP1, 5000, S.ODOMETRY_RESET_4, 12,
                Angle.fromDegrees(90), 0.75, Angle.fromDegrees(180), 200, 300, 3);
        addOdomDrive(S.ODOMETRY_RESET_4, S.DRIVE_WITH_ODOMETRY_5, b, rc, S.STOP1, 5000, S.FOUNDATION_MOVER_UP_RIGHT, 8,
                Angle.fromDegrees(135), 0.75, Angle.fromDegrees(270), 200, 300, 3);
        b.addServo(S.FOUNDATION_MOVER_UP_RIGHT, S.FOUNDATION_MOVER_UP_LEFT, robotCfg.getRightFoundationMover().getName(), SkystoneRobotCfgV2.RightFoundationMoverServoPresets.UP, false);
        b.addServo(S.FOUNDATION_MOVER_UP_LEFT, S.TURN_4, robotCfg.getLeftFoundationMover().getName(), SkystoneRobotCfgV2.LeftFoundationMoverServoPresets.UP, false);
        b.addGyroTurn(S.TURN_4, S.ODOMETRY_RESET_5, Angle.fromDegrees(270));
        addOdomDrive(S.ODOMETRY_RESET_5, S.DRIVE_WITH_ODOMETRY_6, b, rc, S.ODOMETRY_RESET_6, 1000, S.ODOMETRY_RESET_6, -12,
                Angle.fromDegrees(0), 0.75, Angle.fromDegrees(270), 200, 300, 3);
//        b.addDrive(S.LINE_WITH_WALL, StateMap.of(
//                S.MOVE_ARM_DOWN_2, EndConditions.timed(5000),
//                S.MOVE_ARM_DOWN_2, valueBetween(4, minusX, 70, 2)),rc.gyro(270),
//                TranslationControls.sensor2(minusX, 0.07, Angle.fromDegrees(270), new Vector2D(0.75, Angle.fromDegrees(270)), 0.014, 70, 1)
//        );
        addOdomDrive(S.ODOMETRY_RESET_6, S.DRIVE_WITH_ODOMETRY_7, b, rc, S.STOP, 2000, S.STOP, 36,
                Angle.fromDegrees(0), 1, Angle.fromDegrees(270), 200, 300, 3);

//RED auto


        addOdomDrive(null, S.RED_SKYSTONE_RIGHT, b,rc, S.STOP1, 5000, S.RED_MOVE_ARM_UP, 26.5,
                Angle.fromDegrees(227), 1, Angle.fromDegrees(0),
                200, 400, 3);
        addOdomDrive(null, S.RED_SKYSTONE_LEFT, b, rc, S.STOP1, 5000, S.RED_MOVE_ARM_UP, 29, Angle.fromDegrees(248),
                1, Angle.fromDegrees(0), 200, 400, 3);
        addOdomDrive(null, S.RED_SKYSTONE_MIDDLE, b, rc, S.STOP1, 5000, S.RED_MOVE_ARM_UP, 28, Angle.fromDegrees(240),
                1, Angle.fromDegrees(0), 200, 400, 3);

        b.add(S.RED_MOVE_ARM_UP, LiftArmStatesV2.liftMove(S.RED_TURN_1, robotCfg.getLiftArmV2(), 600, false));
        b.addGyroTurn(S.RED_TURN_1, S.RED_START_COLLECTOR, 45, Angle.fromDegrees(2));
        b.addMotorOn(S.RED_START_COLLECTOR,S.RED_ODOMETRY_RESET, robotCfg.getBlockCollector().getCollectorMotor(), -1);
        addOdomDrive(S.RED_ODOMETRY_RESET, S.RED_DRIVE_WITH_ODOMETRY_2,b,rc, S.STOP1, 5000, S.RED_ODOMETRY_RESET_2,
                10, Angle.fromDegrees(135), 0.75, Angle.fromDegrees(45), 150, 300, 3);
        addOdomDrive(S.RED_ODOMETRY_RESET_2, S.RED_DRIVE_WITH_ODOMETRY_3, b,rc,S.STOP1, 5000, S.RED_START_FINGER_RIGHT, -15,
                Angle.fromDegrees(135), 0.75, Angle.fromDegrees(45), 150, 300, 3);
        b.addServo(S.RED_START_FINGER_RIGHT, S.RED_START_FINGER_LEFT, robotCfg.getFingerRight().getName(), SkystoneRobotCfgV2.FingerRightServoPresets.FORWARD, false);
        b.addServo(S.RED_START_FINGER_LEFT, S.RED_MOVE_ARM_DOWN, robotCfg.getFingerLeft().getName(), SkystoneRobotCfgV2.FingerLeftServoPresets.FORWARD, false);
        b.add(S.RED_MOVE_ARM_DOWN, LiftArmStatesV2.liftMove(S.RED_STOP_COLLECTOR, robotCfg.getLiftArmV2(), 0, false));
        b.addMotorOff(S.RED_STOP_COLLECTOR,S.RED_TURN_2, robotCfg.getBlockCollector().getCollectorMotor());
        b.addGyroTurn(S.RED_TURN_2, S.RED_STOP_RIGHT_FINGER, 90, Angle.fromDegrees(1));
        b.addServo(S.RED_STOP_RIGHT_FINGER, S.RED_STOP_FINGER_LEFT, robotCfg.getFingerRight().getName(), SkystoneRobotCfgV2.FingerRightServoPresets.STOP, false);
        b.addServo(S.RED_STOP_FINGER_LEFT, S.RED_CHOOSE_DRIVE_DISTANCE, robotCfg.getFingerLeft().getName(), SkystoneRobotCfgV2.FingerLeftServoPresets.STOP, false);
        b.add(S.RED_CHOOSE_DRIVE_DISTANCE, getDriveState());
        addOdomDrive(S.RED_SKYSTONE_LEFT_DRIVE_TO_BRIDGE, S.RED_DRIVE_WITH_ODOMETRY_8, b,rc,S.STOP1, 5000, S.RED_TURN_3, -70,
                Angle.fromDegrees(180), 1, Angle.fromDegrees(90), 200, 400, 3);
        addOdomDrive(S.RED_SKYSTONE_MIDDLE_TO_BRIDGE, S.RED_DRIVE_WITH_ODOMETRY_9, b,rc,S.STOP1, 5000, S.RED_TURN_3, -67,
                Angle.fromDegrees(180), 1, Angle.fromDegrees(90), 200, 400, 3);
        addOdomDrive(S.RED_SKYSTONE_RIGHT_TO_BRIDGE, S.RED_DRIVE_WITH_ODOMETRY_10, b,rc,S.STOP1, 5000, S.RED_TURN_3, -59,
                Angle.fromDegrees(180), 1, Angle.fromDegrees(90), 200, 400, 3);
        b.addGyroTurn(S.RED_TURN_3, S.RED_FOUNDATION_MOVER_READY_RIGHT, 180, Angle.fromDegrees(2));
        b.addServo(S.RED_FOUNDATION_MOVER_READY_RIGHT, S.RED_FOUNDATION_MOVER_READY_LEFT, robotCfg.getRightFoundationMover().getName(), SkystoneRobotCfgV2.RightFoundationMoverServoPresets.READY, false);
        b.addServo(S.RED_FOUNDATION_MOVER_READY_LEFT, S.RED_DRIVE_DISTANCE_2, robotCfg.getLeftFoundationMover().getName(), SkystoneRobotCfgV2.LeftFoundationMoverServoPresets.READY, false);

        b.addDrive(S.RED_DRIVE_DISTANCE_2, StateMap.of(
                S.RED_MOVE_ARM_UP_FOUNDATION, EndConditions.timed(600),
                S.RED_MOVE_ARM_UP_FOUNDATION, valueBetween(4, plusY, 3, 2)),rc.gyro(180),
                TranslationControls.sensor2(plusY, 0.07, Angle.fromDegrees(90), new Vector2D(0.75, Angle.fromDegrees(-90)), 0.014, 3, 1)
        );
        b.add(S.RED_MOVE_ARM_UP_FOUNDATION, LiftArmStatesV2.liftMove(S.RED_EXTEND_SKYSTONE, robotCfg.getLiftArmV2(), 650, true));
        b.add(S.RED_EXTEND_SKYSTONE, LiftArmStatesV2.horizontalExtension(S.RED_DROP_SKYSTONE, robotCfg.getLiftArmV2(), 1800, true));
        b.addServo(S.RED_DROP_SKYSTONE, S.RED_DROP_SKYSTONE_WAIT, robotCfg.getGripper().getName(), SkystoneRobotCfgV2.GripperServoPresets.RELEASE,  true);
        b.addWait(S.RED_DROP_SKYSTONE_WAIT, S.RED_GRIPPER_READY_RETRACT, 700 );
        b.addServo(S.RED_GRIPPER_READY_RETRACT, S.RED_RETRACT_ARM, robotCfg.getGripper().getName(), SkystoneRobotCfgV2.GripperServoPresets.GRAB, false);
        b.add(S.RED_RETRACT_ARM, LiftArmStatesV2.horizontalExtension(S.RED_FOUNDATION_MOVER_DOWN_RIGHT, robotCfg.getLiftArmV2(), 0, false));
        b.addServo(S.RED_FOUNDATION_MOVER_DOWN_RIGHT, S.RED_FOUNDATION_MOVER_DOWN_LEFT, robotCfg.getRightFoundationMover().getName(), SkystoneRobotCfgV2.RightFoundationMoverServoPresets.DOWN, false);
        b.addServo(S.RED_FOUNDATION_MOVER_DOWN_LEFT, S.RED_ODOMETRY_RESET_3, robotCfg.getLeftFoundationMover().getName(), SkystoneRobotCfgV2.LeftFoundationMoverServoPresets.DOWN, false);
        addOdomDrive(S.RED_ODOMETRY_RESET_3, S.RED_DRIVE_WITH_ODOMETRY_4, b, rc, S.STOP1, 5000, S.RED_MOVE_ARM_DOWN_2, 17,
                Angle.fromDegrees(90), 0.75, Angle.fromDegrees(180), 200, 300, 3);
        b.add(S.RED_MOVE_ARM_DOWN_2, LiftArmStatesV2.liftMove(S.RED_ODOMETRY_RESET_4, robotCfg.getLiftArmV2(), 0, false));
        addOdomDrive(S.RED_ODOMETRY_RESET_4, S.RED_DRIVE_WITH_ODOMETRY_5, b, rc, S.STOP1, 5000, S.RED_FOUNDATION_MOVER_UP_RIGHT, 8,
                Angle.fromDegrees(45), 0.75, Angle.fromDegrees(90), 200, 300, 3);
        b.addServo(S.RED_FOUNDATION_MOVER_UP_RIGHT, S.RED_FOUNDATION_MOVER_UP_LEFT, robotCfg.getRightFoundationMover().getName(), SkystoneRobotCfgV2.RightFoundationMoverServoPresets.UP, false);
        b.addServo(S.RED_FOUNDATION_MOVER_UP_LEFT, S.RED_TURN_4, robotCfg.getLeftFoundationMover().getName(), SkystoneRobotCfgV2.LeftFoundationMoverServoPresets.UP, false);
        b.addGyroTurn(S.RED_TURN_4, S.RED_ODOMETRY_RESET_5, Angle.fromDegrees(90));
        addOdomDrive(S.RED_ODOMETRY_RESET_5, S.RED_DRIVE_WITH_ODOMETRY_6, b, rc, S.RED_ODOMETRY_RESET_6, 1000, S.RED_ODOMETRY_RESET_6, -12,
                Angle.fromDegrees(180), 0.75, Angle.fromDegrees(90), 200, 300, 3);
        addOdomDrive(S.RED_ODOMETRY_RESET_6, S.RED_DRIVE_WITH_ODOMETRY_7, b, rc, S.STOP, 2000, S.STOP, 36,
                Angle.fromDegrees(0), 1, Angle.fromDegrees(90), 200, 300, 3);

        // add dist move here same sensor, to foundation, so 2cm, then drop block
        b.addStop(S.STOP);
        b.addStop(S.STOP1);
        return b.build();
    }


//    private void addOdometryDrive(final StateName resetState, final StateName thisState, EVStateMachineBuilder b, RC rc,
//                                  StateName timeoutState, long timeOutMillis, StateName nextState, double driveInInches, Angle driveAngle,
//                                  double driveSpeed, double deadZone, double targetDelta, int numInARow) {
//        addOdomDrive(resetState, thisState, b, rc, timeoutState, timeOutMillis, nextState, driveInInches,
//        SENSOR_ANGLE, driveSpeed, 0.04, driveAngle, 0.0075, deadZone, targetDelta, numInARow);
//    }


    private void addOdomDrive(final StateName resetState, final StateName thisState, EVStateMachineBuilder b, RC rc,
                              StateName timeoutState, long timeoutMillis, StateName nextState,
                              double driveDistInches, Angle driveDirn, double driveSpeed,
                              Angle orientationGyroAngle,
                              double deadZone, double targetDelta, int numInARow) {
        double odomMinSpeed = 0.04;
        double odomGain = 0.0075;
        addOdomDrive(resetState, thisState, b, rc, timeoutState, timeoutMillis, nextState, driveDistInches, driveDirn, driveSpeed, odomMinSpeed, orientationGyroAngle, odomGain, deadZone, targetDelta, numInARow);
    }


    private void addOdomDrive(final StateName resetOdState, final StateName thisState, EVStateMachineBuilder b,
                              RC rc, StateName timeoutState, long timeoutMillis, StateName nextState,
                              double driveDistInches, Angle driveDirn, double driveSpeed,
                              double minSpeed, Angle orientationGyroAngle,
                              double gain,
                              double deadZone, double targetDelta, int numInARow) {

        final SimpleEncoderSensor odSensor = robotCfg.getOdometryWheelSensor();
        // gain 0.0075
        // minVel 0.04
        // 1 Angle.fromDegrees(285)
        StateMap sm = StateMap.of(
                timeoutState, EndConditions.timed(timeoutMillis),
                nextState, valueBetween(numInARow, odSensor,
                        odSensor.inchesToTicks(driveDistInches), targetDelta));

        Angle sensorPositiveDirection = Angle.fromDegrees(orientationGyroAngle.degrees() + SENSOR_ANGLE.degrees());
        RotationControl rotnCtrl = rc.gyro(orientationGyroAngle.degrees());
        TranslationControl transCtrl = TranslationControls.sensor2(odSensor, gain, sensorPositiveDirection,
                new Vector2D(driveSpeed, driveDirn), minSpeed, odSensor.inchesToTicks(driveDistInches),
                deadZone);

        if (resetOdState != null) {
            b.add(resetOdState, new State() {
                @Override
                public StateName act() {
                    odSensor.reset();
                    return thisState;
                }
            });
        }

        b.addDrive(thisState, sm,rotnCtrl, transCtrl);
    }


//    private void addOdomDrive(final StateName resetOdState, final StateName thisState, EVStateMachineBuilder b,
//                              RC rc, StateName timeoutState, long timeoutMillis, StateName nextState,
//                              double driveDistInches, Angle driveDirn, double driveSpeed,
//                              double minSpeed, Angle orientationGyroAngle,
//                              double gain,
//                              double deadZone, double targetDelta, int numInARow) {
//        final SimpleEncoderSensor odSensor = robotCfg.getOdometryWheelSensor();
//        // gain 0.0075
//        // minVel 0.04
//        // 1 Angle.fromDegrees(285)
//        StateMap sm = StateMap.of(
//                timeoutState, EndConditions.timed(timeoutMillis),
//                nextState, valueBetween(numInARow, odSensor,
//                        odSensor.inchesToTicks(driveDistInches), targetDelta));
//
//        RotationControl rotnCtrl = rc.gyro(orientationGyroAngle.degrees());
//        TranslationControl transCtrl = TranslationControls.sensor2(odSensor, gain, SENSOR_ANGLE,
//                new Vector2D(driveSpeed, driveDirn), minSpeed, odSensor.inchesToTicks(driveDistInches),
//                deadZone);
//
//        if (resetOdState != null) {
//            b.add(resetOdState, new State() {
//                @Override
//                public StateName act() {
//                    odSensor.reset();
//                    return thisState;
//                }
//            });
//        }
//
//        b.addDrive(thisState, sm,rotnCtrl, transCtrl);
//    }

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
                SkyStonePos pos = skystonePosStateRR.getValue();
                if(teamColor == TeamColor.BLUE) {
                    if (SkyStonePos.SKYSTONE_LEFT == pos) {
                        return S.SKYSTONE_LEFT;
                    } else if (SkyStonePos.SKYSTONE_MIDDLE == pos) {
                        return S.SKYSTONE_MIDDLE;
                    } else if (SkyStonePos.SKYSTONE_RIGHT == pos) {
                        return S.SKYSTONE_RIGHT;
                    }
                } else if(teamColor == TeamColor.RED) {
                    if (SkyStonePos.RED_SKYSTONE_LEFT == pos) {
                        return S.RED_SKYSTONE_LEFT;
                    } else if (SkyStonePos.RED_SKYSTONE_MIDDLE == pos) {
                        return S.RED_SKYSTONE_MIDDLE;
                    } else if (SkyStonePos.RED_SKYSTONE_RIGHT == pos) {
                        return S.RED_SKYSTONE_RIGHT;
                    }
                }
                    return S.SKYSTONE_MIDDLE;
            }

        };
    }


    private State getDriveState() {
        return new State() {
            @Override
            public StateName act() {
                SkyStonePos pos = skystonePosStateRR.getValue();
                if(teamColor == TeamColor.BLUE) {
                    if (SkyStonePos.SKYSTONE_LEFT == pos) {
                        return S.SKYSTONE_LEFT_DRIVE_TO_BRIDGE;
                    } else if (SkyStonePos.SKYSTONE_MIDDLE == pos) {
                        return S.SKYSTONE_MIDDLE_TO_BRIDGE;
                    } else if (SkyStonePos.SKYSTONE_RIGHT == pos) {
                        return S.SKYSTONE_RIGHT_TO_BRIDGE;
                    }
                } else if(teamColor == TeamColor.RED) {
                    if (SkyStonePos.RED_SKYSTONE_LEFT == pos) {
                        return S.RED_SKYSTONE_LEFT_DRIVE_TO_BRIDGE;
                    } else if (SkyStonePos.RED_SKYSTONE_MIDDLE == pos) {
                        return S.RED_SKYSTONE_MIDDLE_TO_BRIDGE;
                    } else if (SkyStonePos.RED_SKYSTONE_RIGHT == pos) {
                        return S.RED_SKYSTONE_RIGHT_TO_BRIDGE;
                    }
                }
                return S.SKYSTONE_MIDDLE_TO_BRIDGE;
            }

        };
    }


    private State createCollectState(final StateName nextState, final double collectorSpeed, final long time) {
        return new BasicAbstractState() {
            private long timeNow = System.currentTimeMillis();

            @Override
            public void init() {
            robotCfg.getBlockCollector().setPower(-collectorSpeed);
            }

            @Override
            public boolean isDone() {
                return System.currentTimeMillis() - timeNow == time;
            }

            @Override
            public StateName getNextStateName() {
                return nextState;
            }
        };
    }


    private State createCollectorDriveState(final StateName nextState, double direction,
                                            double orientation, double speed, double distance, final double collectorSpeed,
                                            final double gyroGain) {
        double maxAngSpeed = 0.5;
        final State s = ftc.evlib.statemachine.EVStates.mecanumDrive(nextState,
                Distance.fromFeet(distance), robotCfg.getMecanumControl(),robotCfg.getGyro(), gyroGain, speed,
                Angle.fromDegrees(direction), Angle.fromDegrees(orientation), Angle.fromDegrees(2), maxAngSpeed);

        return new BasicAbstractState() {
            @Override
            public void init() {
                s.act();
//                robotCfg.getFlyWheels().setPower(collectorSpeed);
            }

            @Override
            public boolean isDone() {
                if(s.act() != null) {
  //                  robotCfg.getFlyWheels().stop();
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
        SKYSTONE_DRIVE_TO_LINE,
        SKYSTONE_LEFT,
        SKYSTONE_MIDDLE,
        SKYSTONE_RIGHT,
        RED_SKYSTONE_LEFT,
        RED_SKYSTONE_MIDDLE,
        RED_SKYSTONE_RIGHT,
        SKYSTONE_LEFT_DRIVE_TO_BRIDGE,
        SKYSTONE_MIDDLE_TO_BRIDGE,
        SKYSTONE_RIGHT_TO_BRIDGE,
        RED_SKYSTONE_LEFT_DRIVE_TO_BRIDGE,
        RED_SKYSTONE_RIGHT_TO_BRIDGE,
        RED_SKYSTONE_MIDDLE_TO_BRIDGE,
        DRIVE_1,
        INIT_GYRO, POST_GYRO_WAIT, INIT_CAMERA, POST_CAMERA_PAUSE, WAIT_FOR_START, WAIT_FOR_SKYSTONE, STOP_SKYSTONE_SEARCH, DRIVE_WITH_ODOMETRY, STOP1, DECIDE_SKYSTONE_POSITION, TURN_1, ODOMETRY_RESET, DRIVE_WITH_ODOMETRY_2, COLLECT_SKYSTONE_1, MOVE_ARM_UP, START_COLLECTOR, MOVE_ARM_DOWN, DRIVE_WITH_ODOMETRY_3, START_FINGER_LEFT, START_FINGER_RIGHT, STOP_COLLECTOR, STOP_RIGHT_FINGER, STOP_FINGER_LEFT, ODOMETRY_RESET_2, TURN_2, DRIVE_WITH_ODOMETRY_4, ODOMETRY_RESET_3, TURN_3, DRIVE_MINUS_X, DRIVE_PLUS_Y, DRIVE_DISTANCE_1, EXTEND_SKYSTONE, DRIVE_DISTANCE_2, DRIVE_TIMED_1, GRIPPER_READY_RETRACT, DROP_SKYSTONE, MOVE_ARM_UP_FOUNDATION, INIT_LIFT_ARM, INIT_EXT_ARM, FOUNDATION_MOVER_READY_RIGHT, FOUNDATION_MOVER_READY_LEFT, FOUNDATION_MOVER_DOWN_RIGHT, FOUNDATION_MOVER_DOWN_LEFT, MOVE_FOUNDATION_1, FOUNDATION_MOVER_UP_RIGHT, FOUNDATION_MOVER_UP_LEFT, WAIT_FOR_ARM, DRIVE_DISTANCE_3, ODOMETRY_RESET_4, DRIVE_WITH_ODOMETRY_5, TURN_4, ODOMETRY_RESET_5, DRIVE_WITH_ODOMETRY_6, DROP_SKYSTONE_WAIT, RETRACT_ARM, LINE_WITH_WALL, MOVE_ARM_DOWN_2, ODOMETRY_RESET_6, DRIVE_WITH_ODOMETRY_7, CHOOSE_DRIVE_DISTANCE, DRIVE_WITH_ODOMETRY_8, RED_ODOMETRY_RESET_6, RED_DRIVE_WITH_ODOMETRY_7, RED_MOVE_ARM_UP, RED_TURN_1, RED_START_COLLECTOR, RED_ODOMETRY_RESET, RED_DRIVE_WITH_ODOMETRY_2, RED_ODOMETRY_RESET_2, RED_DRIVE_WITH_ODOMETRY_3, RED_START_FINGER_RIGHT, RED_START_FINGER_LEFT, RED_MOVE_ARM_DOWN, RED_STOP_COLLECTOR, RED_TURN_2, RED_STOP_RIGHT_FINGER, RED_STOP_FINGER_LEFT, RED_CHOOSE_DRIVE_DISTANCE, RED_DRIVE_WITH_ODOMETRY_8, RED_TURN_3, RED_DRIVE_DISTANCE_1, RED_FOUNDATION_MOVER_READY_RIGHT, RED_FOUNDATION_MOVER_READY_LEFT, RED_DRIVE_DISTANCE_2, RED_MOVE_ARM_UP_FOUNDATION, RED_EXTEND_SKYSTONE, RED_DROP_SKYSTONE, RED_DROP_SKYSTONE_WAIT, RED_GRIPPER_READY_RETRACT, RED_DRIVE_DISTANCE_3, RED_FOUNDATION_MOVER_DOWN_RIGHT, RED_DRIVE_WITH_ODOMETRY_9, RED_DRIVE_WITH_ODOMETRY_10, DRIVE_WITH_ODOMETRY_9, DRIVE_WITH_ODOMETRY_10, RED_FOUNDATION_MOVER_DOWN_LEFT, RED_ODOMETRY_RESET_3, RED_ODOMETRY_RESET_4, RED_FOUNDATION_MOVER_UP_RIGHT, RED_FOUNDATION_MOVER_UP_LEFT, RED_TURN_4, RED_DRIVE_WITH_ODOMETRY_4, RED_DRIVE_WITH_ODOMETRY_5, RED_ODOMETRY_RESET_5, RED_DRIVE_WITH_ODOMETRY_6, RED_RETRACT_ARM, RED_LINE_WITH_WALL, RED_MOVE_ARM_DOWN_2, STOP
    }
}
