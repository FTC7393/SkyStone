package org.firstinspires.ftc.teamcode.SkyStone_Season.TestBot;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import ftc.electronvolts.statemachine.StateMachine;
import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.util.InputExtractor;
import ftc.electronvolts.util.TeamColor;
import ftc.electronvolts.util.Vector2D;
import ftc.electronvolts.util.files.Logger;
import ftc.electronvolts.util.units.Angle;
import ftc.electronvolts.util.units.Distance;
import ftc.evlib.hardware.sensors.DistanceSensor;
import ftc.evlib.opmodes.AbstractAutoOp;
import ftc.evlib.statemachine.EVStateMachineBuilder;

@Autonomous
public class TestBotAuto extends AbstractAutoOp<TestBotRobotCfg> {

    private ModernRoboticsI2cRangeSensor range;
    private DistanceSensor pods;


    @Override
    public StateMachine buildStates() {

        EVStateMachineBuilder b = new EVStateMachineBuilder(S.DRIVE_1, TeamColor.BLUE, Angle.fromDegrees(2.0), robotCfg.getGyro(), servos, robotCfg.getMecanumControl());
//        b.addDrive(S.DRIVE_1, S.STOP, Distance.fromFeet(1.0), 0.55, 270, 0);
        b.addDrive(S.DRIVE_1, S.DRIVE_WITH_SENSOR, Distance.fromFeet(0.85), 0.7, 270, 0);
        b.addDriveWithSensor(S.DRIVE_WITH_SENSOR, S.PAUSE, Distance.fromFeet(2), new Vector2D(0.2, Angle.fromDegrees(270)), 0, 0.40, createPodsSR(14), 1.0, 0.02, 0.1);
        b.addWait(S.PAUSE, S.STOP,20000L);
        b.addStop(S.STOP);
        return b.build();
    }

    @Override
    protected TestBotRobotCfg createRobotCfg() {
        return new TestBotRobotCfg(hardwareMap);
    }

    @Override
    public void setup() {
        range = robotCfg.getRange();
        pods = robotCfg.getPods();
        super.setup();
    }

    @Override
    protected Logger createLogger() {
        return null;
    }

    @Override
    protected void setup_act() {
        robotCfg.getRawPods().act();
        telemetry.addData("pods dist", pods.getDistance().centimeters());
        telemetry.addData("raw pods ", robotCfg.getRawPods().getValue());
    }

    @Override
    protected void go() {

    }

    @Override
    protected void act() {
        robotCfg.getRawPods().act();
        telemetry.addData("current state", stateMachine.getCurrentStateName());
        telemetry.addData("  MR dist", robotCfg.getRange().getDistance(DistanceUnit.CM));
        telemetry.addData("pods dist", pods.getDistance().centimeters());
        telemetry.addData("raw pods ", robotCfg.getRawPods().getValue());
    }

    @Override
    protected void end() {

    }

    public enum S implements StateName {
        DRIVE_1,
        DRIVE_WITH_SENSOR,
        PAUSE,
        STOP
    }

    private InputExtractor<Double> createSR(final double distanceInCm) {
        InputExtractor<Double> sensorReading = new InputExtractor<Double>() {
            @Override
            public Double getValue() {
                return range.getDistance(DistanceUnit.CM) - distanceInCm;
            }
        };
        return sensorReading;
    }
    private InputExtractor<Double> createPodsSR(final double distanceInCm) {
        InputExtractor<Double> sensorReading = new InputExtractor<Double>() {
            @Override
            public Double getValue() {
                return pods.getDistance().centimeters() - distanceInCm;
            }
        };
        return sensorReading;
    }

}
