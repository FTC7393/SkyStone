package org.firstinspires.ftc.teamcode.ftctest;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.fair2019.Fair2019RobotCfg;

import ftc.electronvolts.util.Function;
import ftc.electronvolts.util.Functions;
import ftc.electronvolts.util.files.Logger;
import ftc.electronvolts.util.units.Time;
import ftc.evlib.opmodes.AbstractTeleOp;

/**
 * This file was made by the electronVolts, FTC team 7393
 * Date Created: 9/12/16
 * <p>
 * simple TeleOp for the fair robot which has 2 motors
 */
@TeleOp(name = "Tank Drive")
//@Disabled
public class tankDriveFairTeleOp extends AbstractTeleOp<Fair2019RobotCfg> {
   // private DcMotor chest = null;
   // double chestSpeed =.2;
   // boolean chestDirnForward = true;
   // boolean chestRunning = false;

    @Override
    public Time getMatchTime() {
        return Time.fromMinutes(180); //teleop is 2 minutes
    }

    @Override
    protected Fair2019RobotCfg createRobotCfg() {
        return new Fair2019RobotCfg(hardwareMap);
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
       // chest = robotCfg.getChest();

        //take the joystick values and send them to the motors
        robotCfg.getTwoMotors().runMotors(
                driver1.left_stick_y.getValue(),
                driver1.right_stick_y.getValue()
        );

//        if(driver1.x.justPressed()) {
//            chestRunning = !chestRunning;
//        }
//
//        if (chestRunning) {
//            if(driver1.b.justPressed()) {
//                chestDirnForward = !chestDirnForward;
//            }
//            if(driver1.dpad_up.justPressed()) {
//                chestSpeed +=.02;
//            }
//            if(driver1.dpad_down.justPressed()) {
//                chestSpeed -=.02;
//            }
//            chest.setPower(chestSpeed);
//        } else {
//            chest.setPower(0);
//        }
    }

    @Override
    public void end() {

    }

    @Override
    protected Function getJoystickScalingFunction() {
        return Functions.none();
    }
}
