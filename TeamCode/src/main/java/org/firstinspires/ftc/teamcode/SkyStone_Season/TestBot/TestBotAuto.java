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

@Autonomous(name = "TestBotAuto")
public class TestBotAuto extends AbstractAutoOp<TestBotRobotCfg> {

    private ModernRoboticsI2cRangeSensor range;
    private DistanceSensor pods;

    @Override
    public StateMachine buildStates() {



        EVStateMachineBuilder b = new EVStateMachineBuilder(S.DRIVE_1, TeamColor.BLUE, Angle.fromDegrees(2), robotCfg.getGyro(), servos, robotCfg.getMecanumControl());
        b.addDrive(S.DRIVE_1, S.DRIVE_WITH_SENSOR, Distance.fromFeet(0.7), 0.8, 270, 0);
        b.addDriveWithSensor(S.DRIVE_WITH_SENSOR, S.WAIT, Distance.fromFeet(2.0), new Vector2D(0.8, Angle.fromDegrees(270)), 0, 0.30, createSRpods(15), 0.5, 0.03, 0.1);
        b.addWait(S.WAIT, S.STOP, 20000);
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
        super.setup();
    }

    @Override
    protected Logger createLogger() {
        return null;
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
        telemetry.addData("distance from pods", robotCfg.getPods().getValue());
    }

    @Override
    protected void end() {

    }

    public enum S implements StateName {
        DRIVE_1,
        DRIVE_WITH_SENSOR,
        WAIT,
        STOP
    }

    private InputExtractor<Double> createSR(final double distance) {
         InputExtractor<Double> sensorReading = new InputExtractor<Double>() {
            @Override
            public Double getValue() {
                return range.getDistance(DistanceUnit.CM) - distance;
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
