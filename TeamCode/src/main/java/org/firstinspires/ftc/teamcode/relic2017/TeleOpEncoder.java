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
///**
// * Created by ftc7393 on 11/24/2017.
// */
//@TeleOp(name = "TeleOpEncoder")
//
//public class TeleOpEncoder extends AbstractTeleOp<RobotCfg2017> {
//
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
//        InputExtractor<Double> rightY = InputExtractors.negative(driver1.right_stick_y);
//        InputExtractor<Double> leftX = InputExtractors.negative(driver1.left_stick_x);
//        InputExtractor<Double> rightX = driver1.right_stick_x;
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
//        telemetry.addData("liftEncoder", robotCfg.getCollector().getLiftEncoderValue());
//        telemetry.addData("midEncoder", robotCfg.getCollector().getTiltEncoderValue());
//        telemetry.addData("topSwitch", robotCfg.getCollector().getTopLimit());
//        telemetry.addData("bottomSwitch", robotCfg.getCollector().getBottomLimit());
//        telemetry.addData("tiltSwitch", robotCfg.getCollector().getTiltLimit());
//        telemetry.addData("leftTrigger",driver1.left_trigger.getRawValue());
//        telemetry.addData("rightTrigger",driver1.right_trigger.getRawValue());
//        telemetry.addData("mode",robotCfg.getCollector().getMode().name());
//
////        robotCfg.getCollector().setBeltPower(telemetry, driver1.right_trigger.getRawValue(),driver1.left_trigger.getRawValue());
////
////        if(driver1.right_bumper.justPressed()){
////            robotCfg.getCollector().toggleServo();
////        }
////
////        if(driver1.dpad_up.justPressed()){
////            robotCfg.getCollector().goUV();
////        }
////        else if(driver1.dpad_down.justPressed()){
////            robotCfg.getCollector().goDV();
////        }
////        else if(driver1.dpad_left.justPressed()){
////            robotCfg.getCollector().goDH();
////        }
////        else if(driver1.dpad_right.justPressed()){
////            robotCfg.getCollector().goMH();
////        }
//
//        if (driver1.b.justPressed()) {
//            telemetry.addData("button", "dpad right");
//            robotCfg.getCollector().goUp();
//        } else if (driver1.x.justPressed()) {
//            telemetry.addData("button", "dpad left");
//
//            robotCfg.getCollector().goDown();
//        } else if (driver1.y.justPressed()) {
//            telemetry.addData("button", "dpad up");
//
//            robotCfg.getCollector().tiltUp();
//
//        } else if (driver1.a.justPressed()) {
//            telemetry.addData("button", "dpad down");
//
//            robotCfg.getCollector().tiltDown();
//        }
//        else if (!driver1.a.isPressed() && !driver1.b.isPressed() && !driver1.y.isPressed() && !driver1.x.isPressed()) {
//            telemetry.addData("button", "off");
//
//            robotCfg.getCollector().off();
//        }
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
