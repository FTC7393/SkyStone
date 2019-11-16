package org.firstinspires.ftc.teamcode.RoverRuckus;//package org.firstinspires.ftc.teamcode.RoverRuckus;
//
//import com.google.common.collect.ImmutableList;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import evlib.hardware.control.RotationControls;
//import evlib.hardware.control.TranslationControls;
//import evlib.opmodes.AbstractTeleOp;
//import ftc.electronvolts.util.Function;
//import ftc.electronvolts.util.Functions;
//import ftc.electronvolts.util.InputExtractor;
//import ftc.electronvolts.util.InputExtractors;
//import ftc.electronvolts.util.files.Logger;
//import ftc.electronvolts.util.units.Time;
//
///**
// * Created by ftc7393 on 10/13/2018.
// */
//@TeleOp(name = "TestTeleOp")
//
//public class TestTeleOp extends AbstractTeleOp<RoverRuckusRobotCfg>{
//    public Time getMatchTime() {
//        return Time.fromMinutes(180); //teleop is 2 minutes
//    }
//
//    double rotationValue;
//    double extensionValue;
//    ScalingInputExtractor rightY;
//    ScalingInputExtractor leftX;
//    ScalingInputExtractor rightX;
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
//    private MotorSpeedFactor lastXSpeedFactor = currentSpeedFactor;
//
//    enum MotorSpeedFactor {
//        FAST(1.0), SLOW(0.5), SUPER_SLOW(0.2);
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
//    protected RoverRuckusRobotCfg createRobotCfg() {
//        return new RoverRuckusRobotCfg(hardwareMap);
//    }
//
//    @Override
//    protected Logger createLogger() {
//        return new Logger("", "teleop.csv", new ImmutableList.Builder<Logger.Column>()
//
//        .add(new Logger.Column("Rotation Joystick", new InputExtractor<Double>() {
//            @Override
//            public Double getValue() {
//
//
//
//                return getRotationValue();
//            }
//        })) .add(new Logger.Column("Extension Joystick", new InputExtractor<Double>() {
//                    @Override
//                    public Double getValue() {
//
//
//
//                        return getExtensionValue();
//                    }
//                })).addAll(robotCfg.getLoggerColumns()).build());
//
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
//    private void forwardControl() {
//        double f = currentSpeedFactor.getFactor();
//        rightY = new ScalingInputExtractor(driver1.right_stick_y, f);
//        leftX = new ScalingInputExtractor(driver1.left_stick_x, f);
//        rightX = new ScalingInputExtractor(InputExtractors.negative(driver1.right_stick_x), f);
//        //noinspection SuspiciousNameCombination
//        robotCfg.getMecanumControl().setTranslationControl(TranslationControls.inputExtractorXY(rightY, rightX));
////        robotCfg.getMecanumControl().setRotationControl(RotationControls.teleOpGyro(leftX, robotCfg.getGyro()));
//        robotCfg.getMecanumControl().setRotationControl(RotationControls.inputExtractor(leftX));
//    }
//    @Override
//    protected void go() {
//
//
//    }
//
//
//
//    @Override
//    protected void act() {
//        forwardControl();
//
////        telemetry.addData("VelocityR",robotCfg.getMecanumControl().getVelocityR());
////
////        telemetry.addData("VelocityX",robotCfg.getMecanumControl().getVelocityX());
////        telemetry.addData("VelocityY",robotCfg.getMecanumControl().getVelocityY());
////
////        telemetry.addData("rotationJoystick",driver1.right_stick_y.getValue());
//
////
////        telemetry.addData("leftX",driver1.left_stick_x.getValue());
////        telemetry.addData("rightX",driver1.right_stick_x.getValue());
////        telemetry.addData("rightY",driver1.right_stick_y.getValue());
////        telemetry.addData("leftX",driver1.left_stick_x.getValue());
////        telemetry.addData("rightX",driver1.right_stick_x.getValue());
////        telemetry.addData("scalingRightY",rightY.getValue());
////        telemetry.addData("scalingLeftX",leftX.getValue());
//////
////        telemetry.addData("scalingRightX",rightX.getValue());
////testARm
//         rotationValue =driver2.right_stick_y.getValue();
//         extensionValue =driver2.left_stick_y.getValue();
//
//        telemetry.addData("encoder",robotCfg.getArm().getRotationMotor().getEncoderPosition());
//
//        robotCfg.getArm().controlExtension(extensionValue);
//        robotCfg.getArm().controlRotation(rotationValue);
//        telemetry.addData("rotationJoystick",rotationValue);
//
//        telemetry.addData("rotationSetPoint",robotCfg.getArm().getRotationSetPoint());
//        telemetry.addData("a",robotCfg.getArm().getExponential());
//        telemetry.addData("rotationEncoder",robotCfg.getArm().getRotationEncoder());
//        telemetry.addData("extensionEncoder",robotCfg.getArm().getExtensionEncoder());
//
//        if(driver2.left_bumper.isPressed()){
//            robotCfg.getCollector().leftDoor();
//        }
//        else if(driver2.right_bumper.isPressed()) {
//            robotCfg.getCollector().rightDoor();
//        }
//        else if (driver2.right_bumper.isPressed() && driver2.left_bumper.isPressed()) {
//            robotCfg.getCollector().closeDoor();
//
//        }
//        else{
//            robotCfg.getCollector().closeDoor();
//        }
//        if(driver2.x.isPressed()){
//            robotCfg.getCollector().backwardCollection();
//
//        }
//        else if(driver2.y.isPressed()) {
//            robotCfg.getCollector().forwardCollection();
//        }
//        else{
//            robotCfg.getCollector().offCollection();
//        }
//
//
//
//    }
//
//    @Overrideadb usb

//    protected void end() {
//
//    }
//    double getRotationValue(){
//        return rotationValue;
//    }
//    double getExtensionValue(){
//        return extensionValue;
//    }
//
//}
