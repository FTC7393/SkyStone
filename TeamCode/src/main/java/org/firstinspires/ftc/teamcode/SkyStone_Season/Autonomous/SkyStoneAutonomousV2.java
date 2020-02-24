package org.firstinspires.ftc.teamcode.SkyStone_Season.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SkyStone_Season.RobotV2.LiftArmStatesV2;
import org.firstinspires.ftc.teamcode.SkyStone_Season.RobotV2.SkystoneRobotCfgV2;
import org.firstinspires.ftc.teamcode.SkyStone_Season.SkystoneRobotCfg;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
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
import ftc.evlib.hardware.control.RotationControl;
import ftc.evlib.hardware.control.RotationControls;
import ftc.evlib.hardware.control.TranslationControls;
import ftc.evlib.hardware.sensors.AnalogSensor;
import ftc.evlib.hardware.sensors.AveragedSensor;
import ftc.evlib.hardware.sensors.Gyro;
import ftc.evlib.hardware.sensors.SimpleEncoderSensor;
import ftc.evlib.opmodes.AbstractAutoOp;
import ftc.evlib.statemachine.EVStateMachineBuilder;
import ftc.evlib.util.EVConverters;
import ftc.evlib.util.FileUtil;
import ftc.evlib.util.ImmutableList;

@Autonomous(name = "SkyStoneAutoV2")

public class SkyStoneAutonomousV2 extends AbstractAutoOp<SkystoneRobotCfgV2> {
    private static final Angle ODANGLE = Angle.fromDegrees(270);
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
    private boolean doSkyStone = true;
    private Thread cameraInit;
    private ResultReceiver<Boolean> startRR = new BasicResultReceiver<>();
    private double servoSpeed = 1;
    private OpenCvInternalCamera phoneCam;
    private final double encoderTicksPerFoot = 3610;

    @Override
    protected SkystoneRobotCfgV2 createRobotCfg() {
        return new SkystoneRobotCfgV2(hardwareMap);
    }

    @Override
    protected Logger createLogger() {
        return new Logger("log", ".csv", ImmutableList.of(
                new Logger.Column("state", new InputExtractor<String>() {
                    @Override
                    public String getValue() {
                        return stateMachine.getCurrentStateName().name();
                    }
                }),
                new Logger.Column("odometer wheel sensor", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return robotCfg.getOdometryWheelSensor().getValue();
                    }
                }),
                new Logger.Column("velocity", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return TranslationControls.staticV;
                    }
                }),
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
                new Logger.Column("target position", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return robotCfg.getOdometryWheelSensor().inchesToTicks(23);

                    }
                })

        ));

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
        b.addWait(S.POST_GYRO_WAIT, S.INIT_CAMERA, 100);
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
                return S.DECIDE_SKYSTONE_POSITION;
            }
        });
//      b.addDrive(S.DRIVE_WITH_ODOMETRY, S.STOP, Distance.fromFeet(1), 0.8, Angle.fromDegrees(-90), Angle.fromDegrees(0));
        b.add(S.DECIDE_SKYSTONE_POSITION, getSkyStonePosition());
        double odometryDistance = odSensor.inchesToTicks(24);
        b.addDrive(S.SKYSTONE_MIDDLE, StateMap.of(
              S.STOP1, EndConditions.timed(5000),
              S.TURN_1, valueBetween(3, odSensor, odSensor.inchesToTicks(28), 400)
      ), rc.gyro(0), TranslationControls.sensor2(odSensor, 0.0075, ODANGLE,
                new Vector2D(1, Angle.fromDegrees(300)), 0.04, odSensor.inchesToTicks(28), 200));
        b.addGyroTurn(S.TURN_1, S.ODOMETRY_RESET, -45, Angle.fromDegrees(2));
        b.add(S.ODOMETRY_RESET, new State() {
            @Override
            public StateName act() {
                odSensor.reset();
                return S.DRIVE_WITH_ODOMETRY_2;
            }
        });
        double odDistance2 = odSensor.inchesToTicks(5);
        b.addDrive(S.DRIVE_WITH_ODOMETRY_2, StateMap.of(
                S.STOP1, EndConditions.timed(5000),
                S.STOP, valueBetween(3, odSensor,
                        odDistance2, 300)
        ), rc.gyro(-45), TranslationControls.sensor2(odSensor, 0.0075, ODANGLE,
                new Vector2D(0.75, Angle.fromDegrees(-135)), 0.04, odDistance2, 400));
        b.add(S.MOVE_ARM_UP, LiftArmStatesV2.liftMove(S.STOP, robotCfg.getLiftArmV2(), 500, true));
        b.add(S.COLLECT_SKYSTONE_1, createCollectState(S.STOP, 0.7, 2000));
        b.addStop(S.STOP);
        b.addStop(S.STOP1);
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
            robotCfg.getBlockCollector().setPower(collectorSpeed);
            }

            @Override
            public boolean isDone() {
                if(System.currentTimeMillis() - timeNow == time) {
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
        INIT_GYRO, POST_GYRO_WAIT, INIT_CAMERA, POST_CAMERA_PAUSE, WAIT_FOR_START, WAIT_FOR_SKYSTONE, STOP_SKYSTONE_SEARCH, DRIVE_WITH_ODOMETRY, STOP1, DECIDE_SKYSTONE_POSITION, TURN_1, ODOMETRY_RESET, DRIVE_WITH_ODOMETRY_2, COLLECT_SKYSTONE_1, MOVE_ARM_UP, STOP
    }
}
