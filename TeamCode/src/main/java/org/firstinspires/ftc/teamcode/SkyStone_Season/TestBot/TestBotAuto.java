package org.firstinspires.ftc.teamcode.SkyStone_Season.TestBot;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


import ftc.electronvolts.statemachine.EndCondition;
import ftc.electronvolts.statemachine.EndConditions;
import ftc.electronvolts.statemachine.StateMachine;
import ftc.electronvolts.statemachine.StateMap;
import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.util.InputExtractor;
import ftc.electronvolts.util.TeamColor;
import ftc.electronvolts.util.Vector2D;
import ftc.electronvolts.util.files.Logger;
import ftc.electronvolts.util.units.Angle;
import ftc.electronvolts.util.units.Distance;
import ftc.evlib.hardware.control.RotationControl;
import ftc.evlib.hardware.control.RotationControls;
import ftc.evlib.hardware.control.TranslationControls;
import ftc.evlib.hardware.sensors.AnalogSensor;
import ftc.evlib.hardware.sensors.DistanceSensor;
import ftc.evlib.hardware.sensors.Gyro;
import ftc.evlib.opmodes.AbstractAutoOp;
import ftc.evlib.statemachine.EVStateMachineBuilder;
import ftc.evlib.util.ImmutableList;

@Autonomous(name = "TestBotAuto")
public class TestBotAuto extends AbstractAutoOp<TestBotRobotCfg> {

    private static final double DEFAULT_MAX_ANG_SPEED = 0.3;
    private static final Angle DEFAULT_ANGULAR_TOL = Angle.fromDegrees(2.5);
    private ModernRoboticsI2cRangeSensor range;
    private DistanceSensor pods;
    private double sensorInput;



    @Override
    public StateMachine buildStates() {


        AnalogSensor r1 = new AnalogSensor() {
            @Override
            public Double getValue() {
                return robotCfg.getRange().getDistance(DistanceUnit.CM);
            }
        };
        AnalogSensor r2 = new AnalogSensor() {
            @Override
            public Double getValue() {
                return robotCfg.getRange2().getDistance(DistanceUnit.CM);
            }
        };
        AnalogSensor pods = new AnalogSensor() {
            @Override
            public Double getValue() {
                return robotCfg.getPods().getValue();
            }
        };

        Gyro gyro = robotCfg.getGyro();


        EVStateMachineBuilder b = new EVStateMachineBuilder(S.DRIVE_1, TeamColor.BLUE, Angle.fromDegrees(2), robotCfg.getGyro(), servos, robotCfg.getMecanumControl());
        b.addDrive(S.DRIVE_1, S.DRIVE_WITH_SENSOR, Distance.fromFeet(0.7), 0.6, 270, 0, 0.5);

        double target1 = 15.0; // cm
        double tolPods = 1.0;
        double gainPods = 0.02;
        double minVelPods = 0.1;
        Vector2D dwsVel = new Vector2D(0.8, Angle.fromDegrees(270)); // dws = Drive With Sensor
        b.addDrive( S.DRIVE_WITH_SENSOR,
                StateMap.of(
                        S.TURN_1, EndConditions.timed(5000),
                        S.TURN_1, valueCloseTo(pods, target1, 5,tolPods, true)),
                makeStdRC(0),
                TranslationControls.sensor(pods, gainPods, dwsVel, minVelPods, target1));

        b.addDriveWithSensor(S.DRIVE_WITH_SENSOR_OLD, S.STOP, Distance.fromFeet(5.0), dwsVel, 0, 0.50, createSRpods(target1), tolPods, gainPods, minVelPods);

//        b.addDriveWithSensor(S.DRIVE_WITH_SENSOR_2, S.DRIVE_2, Distance.fromFeet(1), new Vector2D(0.8, Angle.fromDegrees(180)), 0, 0.3, createSR(15, robotCfg.getRange2()), 1.0, 0.01, 0.1);
//        b.addWait(S.WAIT, S.STOP, 20000);
        b.addStop(S.STOP);
        b.addStop(S.STOP_2);










//EXPERIMENTAL CODE!!!!!!!!!!
//USE AT YOUR OWN RISK


        b.addGyroTurn(S.TURN_1, S.DRIVE_2A, 0, Angle.fromDegrees(2.5), 0.3);

        b.addDrive(S.DRIVE_2A, S.DRIVE_2B, Distance.fromFeet(2.0), 0.6, 180, 0, 0.5);
        b.addDrive( S.DRIVE_2B,
                StateMap.of(
                  S.STOP, EndConditions.timed(8000),
                  S.STOP_2, valueCloseTo(r2, 15.0, 8,1, true)),
                makeStdRC(0),
                TranslationControls.sensor(r2, 0.015, new Vector2D(0.5, Angle.fromDegrees(180)), 0.1, 15.0));


//        b.addWait(S.DRIVE_2A, S.STOP, 20000);










        return b.build();
    }


    public EndCondition valueCloseTo(InputExtractor<Double> inputExtractor, double target, int numTimes, double tolerance, boolean inclusive) {
        return valueBetween(inputExtractor, numTimes, target - tolerance, target + tolerance, inclusive);
    }
    int numConsecutiveTimes = 0;

    public  EndCondition valueBetween(final InputExtractor<Double> inputExtractor, final int requiredConsecutiveNumTimes, final double min, final double max, final boolean inclusive) {
        return new EndCondition() {
            @Override
            public void init() {
                numConsecutiveTimes = 0;
            }
            @Override
            public boolean isDone() {
                double value = inputExtractor.getValue();
                if (inclusive) {
                    if(min <= value && value <= max) {
                        numConsecutiveTimes++;
                    } else {
                        numConsecutiveTimes = 0;
                    }
                } else {
                    if (min < value && value < max) {
                        numConsecutiveTimes++;
                    } else {
                        numConsecutiveTimes = 0;
                    }
                }
                return numConsecutiveTimes >= requiredConsecutiveNumTimes;
            }
        };
    }

    private RotationControl makeStdRC(double deg) {
        return makeStdRC(deg, DEFAULT_ANGULAR_TOL, DEFAULT_MAX_ANG_SPEED);
    }
    private RotationControl makeStdRC(double deg, Angle angleTol, double maxAngSpeed) {
        return RotationControls.gyro(robotCfg.getGyro(), Angle.fromDegrees(deg), angleTol, maxAngSpeed);
    }

    @Override
    protected TestBotRobotCfg createRobotCfg() {
        return new TestBotRobotCfg(hardwareMap);
    }

    @Override
    public void setup() {
        range = robotCfg.getRange();
        super.setup();
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
                new Logger.Column("range2 values", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return sensorInput;
                    }
                })
        ));
    }

    @Override
    protected void setup_act() {
    }

    @Override
    protected void go() {

    }

    @Override
    protected void act() {
        robotCfg.getPods().act();
        telemetry.addData("current state", stateMachine.getCurrentStateName());
        telemetry.addData("distance from range2",
                robotCfg.getRange2().getDistance(DistanceUnit.CM));
        telemetry.addData("numTimes", numConsecutiveTimes);
        telemetry.addData("distance from pods", robotCfg.getPods().getValue());
    }

    @Override
    protected void end() {

    }

    public enum S implements StateName {
        DRIVE_1,
        DRIVE_WITH_SENSOR,
        DRIVE_WITH_SENSOR_OLD,
//        WAIT,
//        DRIVE_WITH_SENSOR_2,
        STOP_2,
        TURN_1,
        DRIVE_2A, DRIVE_2B, STOP
    }

    private InputExtractor<Double> createSR(final double distance,
                                            final ModernRoboticsI2cRangeSensor range) {
        InputExtractor<Double> sensorReading = new InputExtractor<Double>() {
            @Override
            public Double getValue() {
                sensorInput = range.getDistance(DistanceUnit.CM) - distance;
                return sensorInput;
            }
        };
        return sensorReading;
    }

    private InputExtractor<Double> createSRpods(final double distance) {
        InputExtractor<Double> sensorReading = new InputExtractor<Double>() {
            @Override
            public Double getValue() {
                return robotCfg.getPods().getValue() - distance;
            }
        };
        return sensorReading;
    }

}
