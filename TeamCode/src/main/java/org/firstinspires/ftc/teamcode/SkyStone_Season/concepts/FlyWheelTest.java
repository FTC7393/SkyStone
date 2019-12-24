package org.firstinspires.ftc.teamcode.SkyStone_Season.concepts;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import ftc.electronvolts.util.Function;
import ftc.electronvolts.util.files.Logger;
import ftc.evlib.opmodes.AbstractTeleOp;
@TeleOp(name = "FlyWheel Test OpMode")
@Disabled
public class FlyWheelTest extends AbstractTeleOp<HardwareTestRobotCfg> {


    double savedJoyStickValue = 0;
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

    public void setFlyWheelPower (double newPower) {
        robotCfg.getTwoMotors().runMotors(newPower, -newPower);
    }


    @Override
    protected void act() {
        if(driver1.right_bumper.isPressed()) {
            robotCfg.getTwoMotors().stop();
        }else{
            if(driver1.x.isPressed()) {
                savedJoyStickValue =  driver1.left_stick_x.getValue() * .5;
            }
            setFlyWheelPower(savedJoyStickValue);

        }
        telemetry.addData("collector power", savedJoyStickValue);
    }

    @Override
    protected void end() {

        robotCfg.getTwoMotors().stop();

    }
}
