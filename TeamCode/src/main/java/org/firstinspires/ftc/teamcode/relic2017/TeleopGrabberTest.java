package org.firstinspires.ftc.teamcode.relic2017;//package org.firstinspires.ftc.teamcode.relic2017;
//
//
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import ftc.electronvolts.util.Function;
//import ftc.electronvolts.util.Functions;
//import ftc.electronvolts.util.InputExtractor;
//import ftc.electronvolts.util.InputExtractors;
//import ftc.electronvolts.util.files.Logger;
//import evlib.hardware.control.RotationControls;
//import evlib.hardware.control.TranslationControls;
//import evlib.opmodes.AbstractTeleOp;
//
//
///**
// * Created by ftc7393 on 11/24/2017.
// */
//@TeleOp(name = "Grabber2Test")
//public class TeleopGrabberTest extends AbstractTeleOp<RobotCfg2017> {
//
//    ScalingInputExtractor rightY;
//    ScalingInputExtractor leftX;
//    ScalingInputExtractor rightX;
//
//    class ScalingInputExtractor implements InputExtractor<Double> {
//        InputExtractor<Double> ext;
//        private double factor;
//        ScalingInputExtractor(InputExtractor<Double> ext, double f) {
//            this.ext = ext;
//            this.factor = f;
//        }
//        @Override
//        public Double getValue() {
//            return ext.getValue()*factor;
//        }
//        public void setFactor(double f) {
//            this.factor = f;
//        }
//    }
//    private MotorSpeedFactor currentSpeedFactor = MotorSpeedFactor.FAST;
//
//    enum MotorSpeedFactor {
//        FAST(1.0), SLOW(0.5);
//        private double factor;
//        MotorSpeedFactor(double x) {
//            this.factor = x;
//        }
//        public double getFactor() {
//            return factor;
//        }
//    }
//
//    @Override
//    protected Function getJoystickScalingFunction() {
//        return Functions.eBased(5);
//    }
//
//    @Override
//    protected RobotCfg2017 createRobotCfg() {
//        return new RobotCfg2017(hardwareMap);
//    }
//
//    @Override
//    protected Logger createLogger() {
//        return null;
//    }
//
//    @Override
//    protected void setup() {
//
//    }
//
//    @Override
//    protected void setup_act() {
//
//    }
//
//    private void forwardControl() {
//        double f = currentSpeedFactor.getFactor();
//        rightY = new ScalingInputExtractor(InputExtractors.negative(driver1.right_stick_y), f);
//        leftX = new ScalingInputExtractor(InputExtractors.negative(driver1.left_stick_x), f);
//        rightX = new ScalingInputExtractor(driver1.right_stick_x, f);
//        //noinspection SuspiciousNameCombination
//        robotCfg.getMecanumControl().setTranslationControl(TranslationControls.inputExtractorXY(rightY, rightX));
////        robotCfg.getMecanumControl().setRotationControl(RotationControls.teleOpGyro(leftX, robotCfg.getGyro()));
//        robotCfg.getMecanumControl().setRotationControl(RotationControls.inputExtractor(leftX));
//    }
//
//    @Override
//    protected void go() {
//        forwardControl();
//
//    }
//
//    @Override
//    protected void act() {
//        if(driver1.a.justPressed()) {
//            if (currentSpeedFactor == MotorSpeedFactor.FAST) {
//                currentSpeedFactor = MotorSpeedFactor.SLOW;
//            } else {
//                currentSpeedFactor = MotorSpeedFactor.FAST;
//            }
//            double f = currentSpeedFactor.getFactor();
//            leftX.setFactor(f);
//            rightY.setFactor(f);
//            rightX.setFactor(f);
//        }
//
//        telemetry.addData("liftSetPoint",robotCfg.getCollector().getLiftSetPoint());
//        telemetry.addData("liftEncoder", robotCfg.getCollector().getLiftEncoderValue());
//        telemetry.addData("tiltEncoder", robotCfg.getCollector().getTiltEncoderValue());
//        telemetry.addData("topSwitch", robotCfg.getCollector().getTopLimit());
//        telemetry.addData("bottomSwitch", robotCfg.getCollector().getBottomLimit());
//        telemetry.addData("tiltSwitch", robotCfg.getCollector().getTiltLimit());
//        telemetry.addData("leftTrigger",driver1.left_trigger.getRawValue());
//        telemetry.addData("rightTrigger",driver1.right_trigger.getRawValue());
//        telemetry.addData("mode",robotCfg.getCollector().getMode().name());
//
//        robotCfg.getCollector().setBeltPower(telemetry, driver1.right_trigger.getRawValue(),driver1.left_trigger.getRawValue());
//
//        if(driver1.right_bumper.justPressed()){
//            robotCfg.getCollector().toggleServo();
//        }
//
//        if(driver1.dpad_up.isPressed() && !driver1.dpad_right.isPressed() && !driver1.dpad_left.isPressed()){
//            robotCfg.getCollector().goUV();
//        }
//        else if(driver1.dpad_down.justPressed() && !driver1.dpad_right.isPressed() && !driver1.dpad_left.isPressed()){
//            robotCfg.getCollector().goDV();
//        }
//        else if(driver1.dpad_left.justPressed() && !driver1.dpad_down.isPressed() && !driver1.dpad_up.isPressed()){
//            robotCfg.getCollector().goDH();
//        }
//        else if(driver1.dpad_right.justPressed() && !driver1.dpad_down.isPressed() && !driver1.dpad_up.isPressed()){
//            robotCfg.getCollector().goMH();
//        }
//
////        if(driver1.left_bumper.justPressed()){
////            robotCfg.getCollector().doLittleUp();
////        }
//
//        if(driver1.x.justPressed()){
//            robotCfg.getCollector().unknown();
//        }
//
////        if (driver1.b.justPressed()) {
////            telemetry.addData("button", "dpad right");
////            robotCfg.getCollector().goUp();
////        } else if (driver1.x.justPressed()) {
////            telemetry.addData("button", "dpad left");
////
////            robotCfg.getCollector().goDown();
////        } else if (driver1.y.justPressed()) {
////            telemetry.addData("button", "dpad up");
////
////            robotCfg.getCollector().tiltUp();
////
////        } else if (driver1.a.justPressed()) {
////            telemetry.addData("button", "dpad down");
////
////            robotCfg.getCollector().tiltDown();
////        }
////        //else if (!driver1.a.isPressed() && !driver1.b.isPressed() && !driver1.y.isPressed() && !driver1.x.isPressed()) {
//////            telemetry.addData("button", "off");
//////
//////            robotCfg.getCollector().off();
//////        }
//        robotCfg.getCollector().act();
//
//
//    }
//
//    @Override
//    protected void end() {
//
//    }
//}
