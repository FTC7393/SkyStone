package org.firstinspires.ftc.teamcode.SkyStone_Season.TeleOp;


import org.firstinspires.ftc.teamcode.SkyStone_Season.SkystoneRobotCfg;

import ftc.electronvolts.util.PIDController;
import ftc.evlib.hardware.motors.MotorEnc;
import ftc.evlib.hardware.sensors.DigitalSensor;
import ftc.evlib.hardware.servos.ServoControl;

public class LiftArm {

    private final ServoControl elbow, wrist, fingers;
    private boolean isArmExtended = false;
    private LinearSlide lift = null;
    private final int maxExtensionPosition = 3522; //random number, don't know actual value yet.
    private final int safeArmExtensionPosition = maxExtensionPosition; // lift must be grater than a magic number to extend/retract arm.



    public LiftArm(ServoControl elbow, ServoControl wrist, ServoControl fingers, MotorEnc extension,
                   DigitalSensor lowerLimit, DigitalSensor upperLimit ) {
        this.elbow = elbow;
        this.wrist = wrist;
        this.fingers = fingers;
        this.lift = new LinearSlide(extension, new PIDController(0.0025, 0, 0, 1),
              maxExtensionPosition, lowerLimit, upperLimit);
    }

    public void armPlacing() {
        if (lift.getExtensionEncoder() >=  safeArmExtensionPosition || isArmExtended) {
            elbow.goToPreset(SkystoneRobotCfg.ElbowServoPresets.PLACING);
            wrist.goToPreset(SkystoneRobotCfg.WristServoPresets.PLACING);
            isArmExtended = true;

        }
    }

    public void armPlacingLeft() {
        if (lift.getExtensionEncoder() >=  safeArmExtensionPosition || isArmExtended) {
            elbow.goToPreset(SkystoneRobotCfg.ElbowServoPresets.PLACING);
            wrist.goToPreset(SkystoneRobotCfg.WristServoPresets.PLACING_LEFT);
            isArmExtended = true;

        }
    }

    public void armGrabbing() {
        if (lift.getExtensionEncoder() >=  safeArmExtensionPosition) {
            elbow.goToPreset(SkystoneRobotCfg.ElbowServoPresets.GRABBING);
            wrist.goToPreset(SkystoneRobotCfg.WristServoPresets.GRABBING);
            isArmExtended = false;

        }
    }

    public void armStowed() {
        if (lift.getExtensionEncoder() >=  safeArmExtensionPosition) {
            elbow.goToPreset(SkystoneRobotCfg.ElbowServoPresets.STOWED);
            wrist.goToPreset(SkystoneRobotCfg.WristServoPresets.STOWED);
            isArmExtended = false;

        }
    }

    public void grab() {
        fingers.goToPreset(SkystoneRobotCfg.FingersServoPresets.GRAB);
    }

    public void release() { fingers.goToPreset(SkystoneRobotCfg.FingersServoPresets.RELEASE); }

    public LinearSlide getLift() {
        return lift;
    }

    public void pre_act() {
        lift.pre_act();
    }

    public void act() {
        elbow.act();
        wrist.act();
        fingers.act();
        lift.act();
    }

    public void stop() {
        lift.stopExtension();
    }

}