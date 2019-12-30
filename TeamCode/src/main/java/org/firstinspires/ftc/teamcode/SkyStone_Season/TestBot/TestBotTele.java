package org.firstinspires.ftc.teamcode.SkyStone_Season.TestBot;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import ftc.electronvolts.util.Function;
import ftc.electronvolts.util.Functions;
import ftc.electronvolts.util.files.Logger;
import ftc.evlib.opmodes.AbstractTeleOp;

@TeleOp
public class TestBotTele extends AbstractTeleOp<TestBotRobotCfg> {

//    DistanceSensor distanceSensor;
    ModernRoboticsI2cRangeSensor range;


    @Override
    protected Function getJoystickScalingFunction() {
        return Functions.none();
    }

    @Override
    protected TestBotRobotCfg createRobotCfg() {
        return new TestBotRobotCfg(hardwareMap);
    }

    @Override
    protected Logger createLogger() {
        return null;
    }

    @Override
    protected void setup() {
//        distanceSensor = robotCfg.getDistanceSensor();
        range = robotCfg.getRange();
    }

    @Override
    protected void setup_act() {

    }

    @Override
    protected void go() {

    }

    @Override
    protected void act() {
        int m = robotCfg.getMecanumControl().getMecanumMotors().getEncoder(0);
        int m1 = robotCfg.getMecanumControl().getMecanumMotors().getEncoder(1);
        int m2 = robotCfg.getMecanumControl().getMecanumMotors().getEncoder(2);
        int m3 = robotCfg.getMecanumControl().getMecanumMotors().getEncoder(3);
//        telemetry.addData("distance", distanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("modern robotics distance", range.cmUltrasonic());
        telemetry.addData("motor 0", m);
        telemetry.addData("motor 1", m1);
        telemetry.addData("motor 2", m2);
        telemetry.addData("motor 3", m3);
    }

    @Override
    protected void end() {

    }
}
