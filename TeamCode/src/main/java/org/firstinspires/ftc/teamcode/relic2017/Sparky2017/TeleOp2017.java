package org.firstinspires.ftc.teamcode.relic2017.Sparky2017;


import com.google.common.collect.ImmutableList;
import com.google.common.io.BaseEncoding;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;

import org.firstinspires.ftc.teamcode.relic2017.TeleOpPlayback;

import evlib.hardware.control.RotationControls;
import evlib.hardware.control.TranslationControls;
import evlib.opmodes.AbstractTeleOp;
import evlib.util.StepTimer;
import ftc.electronvolts.util.Function;
import ftc.electronvolts.util.Functions;
import ftc.electronvolts.util.InputExtractor;
import ftc.electronvolts.util.InputExtractors;
import ftc.electronvolts.util.files.Logger;


/**
 * Created by ftc7393 on 11/24/2017.
 */
@TeleOp(name = "TeleOp2017")
@Disabled
public class TeleOp2017 extends AbstractTeleOp<RobotCfg2017> {

    /**
     * the set point change made (eitehr up or down) when the
     * second driver presses the d-pad up or down
     */
    private final int VERT_LIFT_DELTA = 20;
    private enum FlipMode {
        SINGLE_MODE,
        JOINT_MODE
    }

    private FlipMode flipMode = FlipMode.JOINT_MODE;


    ScalingInputExtractor rightY;
    ScalingInputExtractor leftX;
    ScalingInputExtractor rightX;
    Boolean collectorServosIndivdiual =false;

    class ScalingInputExtractor implements InputExtractor<Double> {
        InputExtractor<Double> ext;
        private double factor;
        ScalingInputExtractor(InputExtractor<Double> ext, double f) {
            this.ext = ext;
            this.factor = f;
        }
        @Override
        public Double getValue() {
            return ext.getValue()*factor;
        }
        public void setFactor(double f) {
            this.factor = f;
        }
    }
    private MotorSpeedFactor currentSpeedFactor = MotorSpeedFactor.FAST;
    private MotorSpeedFactor lastXSpeedFactor = currentSpeedFactor;

    enum MotorSpeedFactor {
        FAST(1.0), SLOW(0.5), SUPER_SLOW(0.2);
        private double factor;
        MotorSpeedFactor(double x) {
            this.factor = x;
        }
        public double getFactor() {
            return factor;
        }
    }

    @Override
    protected Function getJoystickScalingFunction() {
        return Functions.eBased(5);
    }

    @Override
    protected RobotCfg2017 createRobotCfg() {
        return new RobotCfg2017(hardwareMap);
    }

    @Override
    protected Logger createLogger() {
        //log the gamepads, and the motors, sensors, servos, etc. from MainRobotCfg
        return new Logger("", "teleop.csv", new ImmutableList.Builder<Logger.Column>()
//                .add(
//                new Logger.Column("state", new InputExtractor<StateName>() {
//                    @Override
//                    public StateName getValue() {
//                        return stateMachine.getCurrentStateName();
//                    }
//                }))
                .addAll(robotCfg.getLoggerColumns()).add(
                        new Logger.Column(TeleOpPlayback.GAMEPAD_1_TITLE, new InputExtractor<String>() {
                            @Override
                            public String getValue() {
                                try {
                                    return BaseEncoding.base64Url().encode(gamepad1.toByteArray());
                                } catch (RobotCoreException e) {
                                    e.printStackTrace();
                                    return "";
                                }
                            }
                        })).add(
                        new Logger.Column(TeleOpPlayback.GAMEPAD_2_TITLE, new InputExtractor<String>() {
                            @Override
                            public String getValue() {
                                try {
                                    return BaseEncoding.base64Url().encode(gamepad2.toByteArray());
                                } catch (RobotCoreException e) {
                                    e.printStackTrace();
                                    return "";
                                }
                            }
                        })
                ).build());
    }


    @Override
    protected void setup() {

    }

    @Override
    protected void setup_act() {

    }

    private void forwardControl() {
        double f = currentSpeedFactor.getFactor();
        rightY = new ScalingInputExtractor(InputExtractors.negative(driver1.right_stick_y), f);
        leftX = new ScalingInputExtractor(InputExtractors.negative(driver1.left_stick_x), f);
        rightX = new ScalingInputExtractor(driver1.right_stick_x, f);
        //noinspection SuspiciousNameCombination
        robotCfg.getMecanumControl().setTranslationControl(TranslationControls.inputExtractorXY(rightY, rightX));
//        robotCfg.getMecanumControl().setRotationControl(RotationControls.teleOpGyro(leftX, robotCfg.getGyro()));
        robotCfg.getMecanumControl().setRotationControl(RotationControls.inputExtractor(leftX));
    }

    @Override
    protected void go() {
        forwardControl();

    }
    StepTimer stepTimer =new StepTimer("TeleOp2017",1);
    @Override
    protected void act() {


        stepTimer.step("");
        MotorSpeedFactor thisSpeedFactor = null;
        if(driver1.right_stick_button.isPressed()) {
            thisSpeedFactor = MotorSpeedFactor.SUPER_SLOW;
        } else if(driver1.left_stick_button.justPressed()) {
            if (currentSpeedFactor == MotorSpeedFactor.FAST) {
                thisSpeedFactor = MotorSpeedFactor.SLOW;
            } else {
                thisSpeedFactor = MotorSpeedFactor.FAST;
            }
            lastXSpeedFactor = thisSpeedFactor;
        } else {
            thisSpeedFactor = lastXSpeedFactor;
        }
        if (thisSpeedFactor != currentSpeedFactor) {
            currentSpeedFactor = thisSpeedFactor;
            double f = currentSpeedFactor.getFactor();
            leftX.setFactor(f);
            rightY.setFactor(f);
            rightX.setFactor(f);

            currentSpeedFactor = thisSpeedFactor;
        }

//        double point=(driver2.right_stick_y.getValue()*10)+robotCfg.getGrabber().getSetPoint();
        telemetry.addData("liftEncoder", robotCfg.getGrabber().getLiftEncoderValue());
        telemetry.addData("topSwitch", robotCfg.getGrabber().getTopLimit());
        telemetry.addData("bottomSwitch", robotCfg.getGrabber().getBottomLimit());
        telemetry.addData("leftTrigger",driver1.left_trigger.getRawValue());
        telemetry.addData("rightTrigger",driver1.right_trigger.getRawValue());
//        telemetry.addData("encoderValue",robotCfg.getGrabber().getLiftEncoderValue());
        telemetry.addData("setPoint",robotCfg.getGrabber().getSetPoint());
        telemetry.addData("encoderValue",robotCfg.getGrabber().getFlipEncoderValue());
        telemetry.addData("flipLIMIT",robotCfg.getGrabber().getFlipLimit());

//        telemetry.addData("potentiometer",robotCfg.getGrabber().getPotentiometer());

//        telemetry.addData("point",point);



//        if(driver1.right_bumper.isPressed()){
//            robotCfg.getGrabber().grabServo();
//        } else if (driver1.left_bumper.isPressed()) {
//            robotCfg.getGrabber().ejectServo();
//        } else {
//            robotCfg.getGrabber().haltServo();
//        }

//
//        if(driver1.right_bumper.justPressed()){
//            robotCfg.getGrabber().toggleServo();
//        }
//        if(driver1.left_bumper.justPressed()){
//            robotCfg.getGrabber().ejectServo();
//        }

        if(driver1.dpad_up.isPressed() && !driver1.dpad_right.isPressed() && !driver1.dpad_left.isPressed()){
            robotCfg.getGrabber().goTop();
        }
        else if(driver1.dpad_down.justPressed() && !driver1.dpad_right.isPressed() && !driver1.dpad_left.isPressed()){
            robotCfg.getGrabber().goBottom();
        }
        else if(driver1.dpad_right.justPressed() && !driver1.dpad_down.isPressed() && !driver1.dpad_left.isPressed()){
            robotCfg.getGrabber().goMiddle();
        }
        else if(driver1.dpad_left.justPressed() && !driver1.dpad_down.isPressed() && !driver1.dpad_right.isPressed()){
            robotCfg.getGrabber().goLittleUp();
        }
        robotCfg.getRelic().getInLimit();
        robotCfg.getRelic().setPower(-driver2.left_stick_y.getValue());

        boolean d2_right_bumper_justPressed = driver2.right_bumper.justPressed();
        if(d2_right_bumper_justPressed){
            robotCfg.getRelic().openServo();
        }
        else if(driver2.left_bumper.justPressed()){
            robotCfg.getRelic().grabServo();
        }
        if(driver2.dpad_up.isPressed()) {
            double newPoint =  VERT_LIFT_DELTA + robotCfg.getGrabber().getSetPoint();
            robotCfg.getGrabber().goPoint(newPoint);
        }
        else if(driver2.dpad_down.isPressed()) {
            double newPoint = robotCfg.getGrabber().getSetPoint() - VERT_LIFT_DELTA;
            robotCfg.getGrabber().goPoint(newPoint);
        }
//        robotCfg.getGrabber().goFlip(driver2.right_stick_y.getValue());


        boolean d1_left_bumper_isPressed = driver1.left_bumper.isPressed();
        boolean d1_right_bumper_isPressed = driver1.right_bumper.isPressed();
//        if(driver1.a.justPressed()){
//            boolean d1_left_bumper_justPressed = driver1.left_bumper.justPressed();
//            if (flipMode == FlipMode.JOINT_MODE ) {
//                if(d1_left_bumper_justPressed && d2_right_bumper_justPressed){
//                    flipMode=FlipMode.SINGLE_MODE;
//                    if(d1_left_bumper_isPressed){
//                        robotCfg.getGrabber().topLeftServo();
//                    }
//                    if(d1_right_bumper_isPressed){
//                        robotCfg.getGrabber().topRightServo();
//                    }
//                    if(driver1.left_trigger.getRawValue()>=.1){
//                        robotCfg.getGrabber().bottomLeftServo();
//                    }
//                    if(driver1.right_trigger.getRawValue()>=.1){
//                        robotCfg.getGrabber().bottomRightServo();
//                    }
//                }
//            } else if (flipMode==FlipMode.SINGLE_MODE) {
//                if(d1_left_bumper_justPressed && d2_right_bumper_justPressed){
//                    flipMode=FlipMode.JOINT_MODE;
//
//                    if(d1_right_bumper_isPressed){
//                        robotCfg.getGrabber().topGrabServo();
//                    } else if(driver1.right_trigger.getRawValue()>=.1){
//                        robotCfg.getGrabber().topEjectServo();
//                    }
//                    if(d1_left_bumper_isPressed){
//                        robotCfg.getGrabber().bottomGrabServo();
//                    }
//                    else if(driver1.left_trigger.getRawValue()>=.1){
//                        robotCfg.getGrabber().bottomEjectServo();
//                    }
//                }
//
//            }
//
//        }

        if(driver2.x.isPressed()){
            double newPoint = robotCfg.getGrabber().getFlipSetPoint() - VERT_LIFT_DELTA;
            robotCfg.getGrabber().goFlipPoint(newPoint);
        }
        if(driver2.y.isPressed()){
            double newPoint =  VERT_LIFT_DELTA + robotCfg.getGrabber().getFlipSetPoint();
            robotCfg.getGrabber().goFlipPoint(newPoint);
        }
        if(driver2.a.isPressed()){
            robotCfg.getGrabber().goFlipUp();
        }
        if(driver2.b.isPressed()){
           robotCfg.getGrabber().goFlipDown();
        }
//
//        if(driver1.x.justPressed()){
//            collectorServosIndivdiual =false;
//        }else if(driver1.y.justPressed()){
//            collectorServosIndivdiual =true;
//        }

//        if(collectorServosIndivdiual ==true){
//                if(d1_left_bumper_isPressed){
//                    robotCfg.getGrabber().topLeftServo();
//                }
//                else {
//                    robotCfg.getGrabber().topLeftServoHalt();
//                }
//                if(d1_right_bumper_isPressed){
//                    robotCfg.getGrabber().topRightServo();
//                }
//                else{
//                    robotCfg.getGrabber().topRightServoHalt();
//                }
//                if(driver1.left_trigger.getRawValue()>=.1){
//                    robotCfg.getGrabber().bottomLeftServo();
//                }
//                else {
//                    robotCfg.getGrabber().bottomLeftServoHalt();
//                }
//                if(driver1.right_trigger.getRawValue()>=.1){
//                    robotCfg.getGrabber().bottomRightServo();
//                }
//                else {
//                    robotCfg.getGrabber().bottomRightServoHalt();
//                }
//        }
//        if(collectorServosIndivdiual ==false){




            if (d1_right_bumper_isPressed) {
                robotCfg.getGrabber().topGrabServo();
            } else if (d1_left_bumper_isPressed) {
                robotCfg.getGrabber().topEjectServo();
            } else {
                robotCfg.getGrabber().topHaltServo();
            }
            if (driver1.right_trigger.getRawValue() >= .1) {
                robotCfg.getGrabber().bottomGrabServo();
            } else if (driver1.left_trigger.getRawValue() >= .1) {
                robotCfg.getGrabber().bottomEjectServo();
            } else {
                robotCfg.getGrabber().bottomHaltServo();
            }



//        }


//        }
















    }

    @Override
    protected void end() {

    }
}
