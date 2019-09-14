package org.firstinspires.ftc.teamcode.RoverRuckus;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import ftc.evlib.opmodes.AbstractTeleOp;
import ftc.evlib.util.StepTimer;
import ftc.electronvolts.util.Function;
import ftc.electronvolts.util.Functions;
import ftc.electronvolts.util.files.Logger;
import ftc.electronvolts.util.units.Time;

/**
 * Created by ftc7393 on 10/13/2018.
 */
@TeleOp(name = "HangingOp")

public class HangingOp extends AbstractTeleOp<RoverRuckusRobotCfg> {

    public Time getMatchTime() {
        return Time.fromMinutes(3); //teleop is 2 minutes
    }

    int backFootState = 0;
    boolean latch = true;
    boolean markerOn = true;
    double rotationValue;
    double extensionValue;




    @Override
    protected Function getJoystickScalingFunction() {
        return Functions.eBased(5);
    }

    @Override
    protected RoverRuckusRobotCfg createRobotCfg() {
        return new RoverRuckusRobotCfg(hardwareMap);
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
        robotCfg.getHanging().unlatch();
        robotCfg.getPhonePan().goToPreset(RoverRuckusRobotCfg.PhonePanPresets.MIDDLE);

    }



    @Override
    protected void go() {


    }


    StepTimer teleOpTimer = new StepTimer("teleOp", Log.INFO);

    @Override
    protected void act() {
        teleOpTimer.start();

//        telemetry.addData("upLimit",robotCfg.getHanging().latchLimit.getValue());
//        telemetry.addData("downLimit",robotCfg.getHanging().unlatchLimit.getValue());

        teleOpTimer.step("Hanging/Latch/Marker");

        if (driver1.dpad_up.isPressed()) {
            robotCfg.getHanging().noLimUpHanging();
        } else if (driver1.dpad_down.isPressed()) {

            robotCfg.getHanging().noLimDownHanging();
        } else {
            robotCfg.getHanging().stopHanging();

        }


        if (driver1.b.justPressed()) {
            if (latch == true) {
                //telemetry.addLine("latch");

                robotCfg.getHanging().latch();
                latch = false;

            } else if (latch == false) {
                //telemetry.addLine("unlatch");

                robotCfg.getHanging().unlatch();
                latch = true;
            }
        }

//g.getArm().rotationMaxPosition);


        teleOpTimer.stop();

    }

    @Override
    protected void end() {

    }



}
