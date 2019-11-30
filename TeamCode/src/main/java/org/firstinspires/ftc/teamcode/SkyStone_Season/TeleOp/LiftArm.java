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
    public enum WristPositions {
        STRAIGHT,
        LEFT,
        RIGHT,
        RETRACTED
    }
    private WristPositions wristPosition = WristPositions.STRAIGHT;

    public LiftArm(ServoControl elbow, ServoControl wrist, ServoControl fingers, MotorEnc extension,
                   DigitalSensor lowerLimit, DigitalSensor upperLimit ) {
        this.elbow = elbow;
        this.wrist = wrist;
        this.fingers = fingers;
        this.lift = new LinearSlide(extension, new PIDController(0.0025, 0, 0, 1),
              maxExtensionPosition, lowerLimit, upperLimit);
    }

    public void armExtend() {
        if (lift.getExtensionEncoder() >=  safeArmExtensionPosition) {
            elbow.goToPreset(SkystoneRobotCfg.ElbowServoPresets.EXTEND);
            wrist.goToPreset(SkystoneRobotCfg.WristServoPresets.EXTEND);
            isArmExtended = true;
            wristPosition = WristPositions.STRAIGHT;
        }
    }

    public void armRetract() {
        if (lift.getExtensionEncoder() >=  safeArmExtensionPosition) {
            elbow.goToPreset(SkystoneRobotCfg.ElbowServoPresets.RETRACT);
            wrist.goToPreset(SkystoneRobotCfg.WristServoPresets.RETRACT);
            isArmExtended = false;
            wristPosition = WristPositions.RETRACTED;
        }
    }


    public void wristRight() {
        if (isArmExtended == true){
            wrist.goToPreset(SkystoneRobotCfg.WristServoPresets.RIGHT);
            wristPosition = WristPositions.RIGHT;
        }
    }

    public void wristStraight() {
        if (isArmExtended == true){
            wrist.goToPreset(SkystoneRobotCfg.WristServoPresets.EXTEND);
            wristPosition = WristPositions.STRAIGHT;
        }
    }

    public void wristLeft() {
        if (isArmExtended == true){
            wrist.goToPreset(SkystoneRobotCfg.WristServoPresets.LEFT);
            wristPosition = WristPositions.LEFT;
        }
    }

    public WristPositions getWristPosition() {
        return wristPosition;
    }

    public void grab() {
        fingers.goToPreset(SkystoneRobotCfg.FingersServoPresets.GRAB);
    }

    public void release() { fingers.goToPreset(SkystoneRobotCfg.FingersServoPresets.RELEASE); }

    public LinearSlide getLift() {
        return lift;
    }

    public void act() {
        elbow.act();
        wrist.act();
        fingers.act();
        lift.act();
    }

}