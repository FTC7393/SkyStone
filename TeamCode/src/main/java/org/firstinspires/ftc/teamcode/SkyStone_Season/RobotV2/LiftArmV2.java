package org.firstinspires.ftc.teamcode.SkyStone_Season.RobotV2;

import org.firstinspires.ftc.teamcode.SkyStone_Season.Autonomous.ProportionalOffsetCalculator;
import org.firstinspires.ftc.teamcode.SkyStone_Season.TeleOp.LinearSlide;

import ftc.electronvolts.statemachine.StateMachine;
import ftc.electronvolts.util.PIDController;
import ftc.evlib.hardware.motors.MotorEnc;
import ftc.evlib.hardware.sensors.DigitalSensor;
import ftc.evlib.hardware.servos.ServoControl;

public class LiftArmV2 {

    private final ServoControl wrist, gripper, fingerLeft, fingerRight;



    private final DigitalSensor lowerLimitVerticalRight;
    private final DigitalSensor lowerLimitVerticalLeft;
    private final DigitalSensor lowerLimitHorizontal;
    private boolean isArmExtended = false;
    private LinearSlide horizontalSlide;
    private LinearSlide verticalSlideRight;
    private LinearSlide verticalSlideLeft;
    private StateMachine stateMachine;
    private ProportionalOffsetCalculator calculator = new ProportionalOffsetCalculator(50, 400, 150);
    private double liftCommand;
    private double WristCommand;
    private final int VerticalMaxExtension = 3600;
    private int VerticalMinExtension = -10000;
    private final int HorizontalMaxExtension = 1850;
    private final int LiftToleranceHorizontal = 5;
    private final int LiftToleranceVertical = 5;
    private final int liftKeepOutUpperLimit = 500;
    private final int liftKeepOutInnerLimit = 20;
    private final int liftKeepOutOuterLimit = 1300;
    private final int WristKeepOutOuterLimit = 750;


    private final double elbowSpeed = 0.6;
    private final double wristSpeed = 0.6;
    private final double fingerSpeed = 1.15;
    private boolean isNinety = false;

    public LinearSlide getVerticalSlideRight() {
        return verticalSlideRight;
    }

    public LinearSlide getVerticalSlideLeft() {
        return verticalSlideLeft;
    }

    public LiftArmV2(ServoControl wrist, ServoControl gripper, ServoControl fingerRight, ServoControl fingerLeft, MotorEnc VerticalRightMotor,
                     MotorEnc VerticalLeftMotor, MotorEnc HorizontalMotor, DigitalSensor lowerLimitVerticalRight, DigitalSensor lowerLimitVerticalLeft, DigitalSensor lowerLimitHorizontal) {
        this.wrist = wrist;
        this.gripper = gripper;
        this.fingerLeft = fingerLeft;
        this.fingerRight = fingerRight;
//        this.verticalSlideLeft = new LinearSlide(VerticalLeftMotor, new PIDController(0.003, 0, 0, 1),
//                VerticalMaxExtension, LiftToleranceVertical, lowerLimitVerticalLeft);
//        this.verticalSlideRight = new LinearSlide(VerticalRightMotor, new PIDController(0.003, 0, 0, 1),
//                VerticalMaxExtension, LiftToleranceVertical, lowerLimitVerticalRight);
//        this.horizontalSlide = new LinearSlide(HorizontalMotor, new PIDController(0.003, 0, 0.1, 1),
//                HorizontalMaxExtension, LiftToleranceHorizontal, lowerLimitHorizontal);
        this.horizontalSlide = new LinearSlide(HorizontalMotor, null,
                HorizontalMaxExtension, LiftToleranceHorizontal, lowerLimitHorizontal);
        horizontalSlide.setMaxCorrectionPower(0.6);
        this.verticalSlideRight = new LinearSlide(VerticalRightMotor, null,
                VerticalMaxExtension, LiftToleranceVertical, lowerLimitVerticalRight);
        verticalSlideRight.setMaxCorrectionPower(0.8);
        this.verticalSlideLeft = new LinearSlide(VerticalLeftMotor, null,
                VerticalMaxExtension, LiftToleranceVertical, lowerLimitVerticalLeft);
        verticalSlideLeft.setMaxCorrectionPower(0.8);
        this.lowerLimitVerticalRight = lowerLimitVerticalRight;
        this.lowerLimitVerticalLeft = lowerLimitVerticalLeft;
        this.lowerLimitHorizontal = lowerLimitHorizontal;
    }
    public DigitalSensor getLowerLimitVerticalLeft() {
        return lowerLimitVerticalLeft;
    }
    public DigitalSensor getLowerLimitHorizontal() {
        return lowerLimitHorizontal;
    }
    public DigitalSensor getLowerLimitVerticalRight() {
        return lowerLimitVerticalRight;
    }


    public void controlExtension(double extensionDelta) {
        double newCommand = horizontalSlide.getExtensionSetPoint() + extensionDelta;
//        if (verticalSlideRight.getExtensionEncoder() < liftKeepOutUpperLimit && verticalSlideLeft.getExtensionEncoder() < liftKeepOutUpperLimit) {
//            if (horizontalSlide.getExtensionSetPoint() <= liftKeepOutInnerLimit && newCommand > liftKeepOutInnerLimit) {
//                newCommand = liftKeepOutInnerLimit;
//            } else if (horizontalSlide.getExtensionSetPoint() >= liftKeepOutOuterLimit && newCommand < liftKeepOutOuterLimit) {
//                newCommand = liftKeepOutOuterLimit;
//            }
//        }
        setExtension(newCommand);
    }

    public void setExtension(double newCommand) {
        if (isNinety) {
            if (horizontalSlide.getExtensionSetPoint() > WristKeepOutOuterLimit && newCommand < WristKeepOutOuterLimit) {
                newCommand = WristKeepOutOuterLimit;
            }
        }
        horizontalSlide.setExtension(newCommand);
    }


    public void freezeExtension() {
        double horizontalEnc = horizontalSlide.getExtensionEncoder();
        setExtension(horizontalEnc);
    }

    public void setLift(int encoder){
        double leftEnc = verticalSlideLeft.getExtensionEncoder();
//        double newCommand = ((leftEnc + verticalSlideRight.getExtensionEncoder()) / 2) + liftDelta;
        double offset = calculator.calculateOffset(leftEnc);
        liftCommand = Math.min(Math.max(encoder, VerticalMinExtension), VerticalMaxExtension);
        //if you want to contrain the lift based on the horizontal condition, here is some logic to try
//        if (horizontalSlide.getExtensionEncoder() >= liftKeepOutInnerLimit && horizontalSlide.getExtensionEncoder() <= liftKeepOutOuterLimit) {
//            if (liftCommand >= liftKeepOutUpperLimit && newCommand < liftKeepOutUpperLimit) {
//                newCommand = liftKeepOutUpperLimit;
//            }
//        }

        verticalSlideLeft.setExtension(Math.min(Math.max(liftCommand-offset, VerticalMinExtension), VerticalMaxExtension));
        verticalSlideRight.setExtension(liftCommand);
    }

    public void controlLift(double liftDelta) {
        double newCommand = liftCommand+liftDelta;

        setLift((int)newCommand);
    }

    public void freezeLift() {
        double rightEnc = verticalSlideRight.getExtensionEncoder();
        setLift((int)rightEnc);
    }

    public void fingersIngest() {
        fingerLeft.goToPreset(SkystoneRobotCfgV2.FingerLeftServoPresets.FORWARD);
        fingerRight.goToPreset(SkystoneRobotCfgV2.FingerRightServoPresets.FORWARD);
    }

    public void fingerEject() {
        fingerLeft.goToPreset(SkystoneRobotCfgV2.FingerLeftServoPresets.BACKWARD);
        fingerRight.goToPreset(SkystoneRobotCfgV2.FingerRightServoPresets.BACKWARD);
    }

    public void fingersRight() {
        fingerLeft.goToPreset(SkystoneRobotCfgV2.FingerLeftServoPresets.BACKWARD);
        fingerRight.goToPreset(SkystoneRobotCfgV2.FingerRightServoPresets.FORWARD);
    }

    public void fingersLeft() {
        fingerLeft.goToPreset(SkystoneRobotCfgV2.FingerLeftServoPresets.FORWARD);
        fingerRight.goToPreset(SkystoneRobotCfgV2.FingerRightServoPresets.BACKWARD);
    }

    public void fingersStop() {
        fingerLeft.goToPreset(SkystoneRobotCfgV2.FingerLeftServoPresets.STOP);
        fingerRight.goToPreset(SkystoneRobotCfgV2.FingerRightServoPresets.STOP);
    }

    public boolean wrist90() {
        if (horizontalSlide.getExtensionEncoder() > WristKeepOutOuterLimit) {
            wrist.goToPreset(SkystoneRobotCfgV2.WristServoPresets.NINETY);
            isNinety = true;
            return true;
        } else {
            return false;
        }
    }

    public void wrist0() {
        wrist.goToPreset(SkystoneRobotCfgV2.WristServoPresets.ZERO);
        isNinety = false;
    }

    public void gripperGrab() {
        gripper.goToPreset(SkystoneRobotCfgV2.GripperServoPresets.GRAB);
    }

    public void gripperRelease() {
        gripper.goToPreset(SkystoneRobotCfgV2.GripperServoPresets.RELEASE);
    }


    public double getLiftPosition() {
        return ((verticalSlideLeft.getExtensionEncoder() + verticalSlideRight.getExtensionEncoder()) / 2);
    }

    public double getVerticalLeftEncoder() {
        return verticalSlideLeft.getExtensionEncoder();
    }

    public double getVerticalRightEncoder() {
        return verticalSlideRight.getExtensionEncoder();
    }

    public double getHorizontalEncoder() {
        return horizontalSlide.getExtensionEncoder();
    }

    public double getLiftCommand(){
        return liftCommand;
    }

    public double getExtensionCommand(){
        return horizontalSlide.getExtensionSetPoint();
    }

    public void setLiftPosition(double liftPosition) {
        verticalSlideLeft.setExtension(liftPosition);
        verticalSlideRight.setExtension(liftPosition);
    }


    public boolean isDone() {
        return horizontalSlide.isDone() && verticalSlideRight.isDone() && verticalSlideLeft.isDone() && wrist.isDone() && gripper.isDone();
    }

    public boolean verticalIsDone() {
       return verticalSlideRight.isDone() && verticalSlideLeft.isDone();
    }

    public boolean horizontalIsDone() {
        return horizontalSlide.isDone();
    }

    public boolean handIsDone() {
       return wrist.isDone() && gripper.isDone();
    }

    public void pre_act() {
        horizontalSlide.pre_act();
        verticalSlideLeft.pre_act();
        verticalSlideRight.pre_act();
        if(verticalSlideLeft.getLowerLimit() && verticalSlideRight.getLowerLimit()) {
            VerticalMinExtension = -50;
        }
    }

    public void act() {
//        stateMachine.act();
        verticalSlideRight.act();
        verticalSlideLeft.act();
        horizontalSlide.act();
        wrist.act();
        gripper.act();
        fingerLeft.act();
        fingerRight.act();
    }

    public void stop(){
        verticalSlideRight.stopExtension();
        verticalSlideLeft.stopExtension();
        horizontalSlide.stopExtension();
    }
}

    /////////////////////////
    ///stateMachineSection///
    /////////////////////////
