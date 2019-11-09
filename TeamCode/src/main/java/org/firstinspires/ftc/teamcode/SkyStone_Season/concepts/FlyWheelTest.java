package org.firstinspires.ftc.teamcode.SkyStone_Season.concepts;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import ftc.electronvolts.util.Function;
import ftc.electronvolts.util.files.Logger;
import ftc.evlib.opmodes.AbstractTeleOp;
@TeleOp(name = "FlyWheelTest")
public class FlyWheelTest extends AbstractTeleOp<HardwareTestRobotCfg> {

    double leftValue = 0.5;
    double rightValue = -0.5;
    @Override
    protected Function getJoystickScalingFunction() {
        return null;
    }

    @Override
    protected HardwareTestRobotCfg createRobotCfg() {
        return new HardwareTestRobotCfg(hardwareMap);
    }

    @Override
    protected Logger createLogger() {
        return null;
    }

    @Override
    protected void setup() {

    }

    @Override
    protected void setup_act() {

    }

    @Override
    protected void go() {

    }

    @Override
    protected void act() {
        if(driver1.right_stick_button.justPressed()) {
            robotCfg.getTwoMotors().runMotors(leftValue, rightValue);
        }

        if(driver1.left_stick_button.justPressed()) {
            robotCfg.getTwoMotors().stop();
        }

    }

    @Override
    protected void end() {

        robotCfg.getTwoMotors().stop();

    }
}
