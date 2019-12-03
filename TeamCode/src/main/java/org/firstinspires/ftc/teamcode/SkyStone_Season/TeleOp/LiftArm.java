package org.firstinspires.ftc.teamcode.SkyStone_Season.TeleOp;


import org.firstinspires.ftc.teamcode.SkyStone_Season.SkystoneRobotCfg;

import ftc.electronvolts.statemachine.StateMachine;
import ftc.electronvolts.statemachine.StateMachineBuilder;
import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.util.PIDController;
import ftc.evlib.hardware.motors.MotorEnc;
import ftc.evlib.hardware.sensors.DigitalSensor;
import ftc.evlib.hardware.servos.ServoControl;

public class LiftArm {

    private final ServoControl elbow, wrist, fingers;
    private boolean isArmExtended = false;
    private LinearSlide lift = null;
    private final int maxExtensionPosition = 3522; //random number, don't know actual value yet. TODO
    private final int liftTolerance = 5;
    private final int safeArmExtensionPosition = maxExtensionPosition; // lift must be grater than a magic number to extend/retract arm.
    private final int emptySafeHeight = 2000; //random number, don't know actual value yet. TODO




    public LiftArm(ServoControl elbow, ServoControl wrist, ServoControl fingers, MotorEnc extension,
                   DigitalSensor lowerLimit, DigitalSensor upperLimit ) {
        this.elbow = elbow;
        this.wrist = wrist;
        this.fingers = fingers;
        this.lift = new LinearSlide(extension, new PIDController(0.0025, 0, 0, 1),
              maxExtensionPosition, liftTolerance, lowerLimit, upperLimit);
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

    public boolean isDone() {
        return lift.isDone() && elbow.isDone() && wrist.isDone() && fingers.isDone();
    }

    public void act() {
        elbow.act();
        wrist.act();
        fingers.act();
        lift.act();
    }

    /////////////////////////
    ///stateMachineSection///
    /////////////////////////

    public StateMachine buildStates(){
        StateMachineBuilder b = new StateMachineBuilder(S.STOWED);
        /*
        Stowed ⟶ Grabbing A1:
        - Move linear slide up to an empty safe height, wait for move to complete
        - Elbow and wrist servos in grabbing position and finger servo is in open position
        - Lover linear slide to a grabbing height, wait for move to complete
        - Put finger servos in a closed position
         */









        return b.build();
    }

    /*
      A1: Stowed ⟶ Grabbing A1:
      A2: Grabbing ⟶ Placing A2:
      A3: Placing ⟶ Dropping A3:
      A4: Dropping ⟶ Placing A4:
      A5: Placing ⟶ Stowed A5:
      B1: Grabbing ⟶ Stowed B1:
      B2: Placing ⟶ Grabbing B2:
      B3: Stowed ⟶ Placing B3:
     */
    public enum S implements StateName {
        STOWED,
        GRABBED,
        PLACING,
        DROPPING,
        MOVE_A1_1,
        MOVE_A1_2,
        MOVE_A1_3,
        MOVE_A1_4,
        MOVE_A1_5,
        MOVE_A1_6
    }
}