package org.firstinspires.ftc.teamcode.Sri_TestBot_Skystone;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Abhi_Robot.TestBotCfg;

import ftc.electronvolts.util.Function;
import ftc.electronvolts.util.Functions;
import ftc.electronvolts.util.files.Logger;
import ftc.electronvolts.util.units.Time;
import ftc.evlib.hardware.servos.ServoControl;
import ftc.evlib.opmodes.AbstractTeleOp;

/**
 * This file was made by the electronVolts, FTC team 7393
 * Date Created: 9/12/16
 * <p>
 * simple TeleOp for the fair robot which has 2 motors
 */
@TeleOp(name = "Tank Drive Robot TeleOp")
//@Disabled
public class TankDriveTeleOp2019 extends AbstractTeleOp<TestBotCfg> {
    private ServoControl clamp = null;
    boolean clampIsClosed = true;

    @Override
    public Time getMatchTime() {
        return Time.fromMinutes(180); //teleop is 2 minutes
    }

    @Override
    protected TestBotCfg createRobotCfg() {
        return new TestBotCfg(hardwareMap);
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
        clamp = robotCfg.getClamp();
    }

    @Override
    protected void go() {
    }

    @Override
    protected void act() {
        boolean closeClamp;

        //take the joystick values and send them to the motors
        robotCfg.getTwoMotors().runMotors(
                driver1.left_stick_y.getValue(),
                driver1.right_stick_y.getValue()
        );



        if(driver1.left_bumper.justPressed()) {
            closeClamp = false;
        } else if(driver1.right_bumper.justPressed()) {
            closeClamp = true;
        } else {
            closeClamp = clampIsClosed;
        }

        if(closeClamp != clampIsClosed) {
            if(closeClamp) {
                clamp.goToPreset(TestBotCfg.ClampServoPresets.CLOSED);
            } else {
                clamp.goToPreset(TestBotCfg.ClampServoPresets.OPEN);
            }
        }

        clampIsClosed = closeClamp; //save clamp state for next pass

    }

    @Override
    public void end() {

    }

    @Override
    protected Function getJoystickScalingFunction() {
        return Functions.none();
    }
}
