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
import ftc.evlib.opmodes.AbstractAutoOp;
import ftc.evlib.statemachine.EVStateMachineBuilder;

@Autonomous
public class TestBotAuto extends AbstractAutoOp<TestBotRobotCfg> {

    private ModernRoboticsI2cRangeSensor range;


    @Override
    public StateMachine buildStates() {



        EVStateMachineBuilder b = new EVStateMachineBuilder(S.DRIVE_1, TeamColor.BLUE, Angle.fromDegrees(4), robotCfg.getGyro(), servos, robotCfg.getMecanumControl());
        b.addDrive(S.DRIVE_1, S.DRIVE_WITH_SENSOR, Distance.fromFeet(1.0), 0.25, 270, 0);
        b.addDriveWithSensor(S.DRIVE_WITH_SENSOR, S.STOP, Distance.fromFeet(2), new Vector2D(0.8, Angle.fromDegrees(270)), 0, 0.30, createSR(4), 1.0, 1.0, 0.1);
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
        telemetry.addData("current state", stateMachine.getCurrentStateName());
    }

    @Override
    protected void end() {

    }

    public enum S implements StateName {
        DRIVE_1,
        DRIVE_WITH_SENSOR,
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

}
