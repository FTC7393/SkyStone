package org.firstinspires.ftc.teamcode.SkyStone_Season.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.SkyStone_Season.SkystoneRobotCfg;

import ftc.electronvolts.util.Function;
import ftc.electronvolts.util.Functions;
import ftc.electronvolts.util.InputExtractor;
import ftc.electronvolts.util.InputExtractors;
import ftc.electronvolts.util.files.Logger;
import ftc.electronvolts.util.units.Time;
import ftc.evlib.hardware.control.RotationControls;
import ftc.evlib.hardware.control.TranslationControls;
import ftc.evlib.hardware.servos.ServoControl;
import ftc.evlib.opmodes.AbstractTeleOp;
//comment
/**
 * Created by ftc7393 on 9/22/2018.
 */
@TeleOp(name = "SkyStone Tele")

public class SkystoneTeleOp extends AbstractTeleOp<SkystoneRobotCfg> {
    private ServoControl dump = null;
    private DcMotor collector = null;
    private FlyWheels flywheels = null;
    int dumpPosition;
    boolean driver1CollectorEnabled = true;


    @Override
    public Time getMatchTime() {
        return Time.fromMinutes(180); //teleop is 2 minutes
    }


    ScalingInputExtractor rightY;
    ScalingInputExtractor leftX;
    ScalingInputExtractor rightX;
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
    protected SkystoneRobotCfg createRobotCfg() {
        return new SkystoneRobotCfg(hardwareMap);
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

    }
    private void forwardControl() {
        double f = currentSpeedFactor.getFactor();
        rightY = new ScalingInputExtractor(driver1.right_stick_y, f);
        leftX = new ScalingInputExtractor(driver1.left_stick_x, f);
        rightX = new ScalingInputExtractor(InputExtractors.negative(driver1.right_stick_x), -1.0*f);
        robotCfg.getMecanumControl().setTranslationControl(TranslationControls.inputExtractorXY(rightY, rightX));
//        robotCfg.getMecanumControl().setRotationControl(RotationControls.teleOpGyro(leftX, robotCfg.getGyro()));
        robotCfg.getMecanumControl().setRotationControl(RotationControls.inputExtractor(leftX));
    }
    @Override
    protected void go() {


    }

    @Override
    protected void act() {

        forwardControl();
        dump  = null; // robotCfg.getServo(FutureFestRobotCfg.FutureFestServoEnum.DUMP_SERVO);

        if(driver2.x.isPressed()) {
            driver1CollectorEnabled = false;
        } else {
            driver1CollectorEnabled = true;
        }

        // Collector logic: Driver1 has control and can press right bumper for intake or left bumper for output
        // But is the right trigger is pressed in different levels (0.9, 0.6 and 0.3) it can set the collector power to different speeds
        if(driver1.right_bumper.isPressed() && !driver1.left_bumper.isPressed())  {
            robotCfg.getFlyWheels().setPower(0.45);
        } else if(driver1.left_bumper.isPressed() && !driver1.right_bumper.isPressed()) {
            robotCfg.getFlyWheels().setPower(-0.45);
        }else if (driver1.right_trigger.getValue() >= 0.95) {
            robotCfg.getFlyWheels().setPower(1.0);
        }else if (driver1.right_trigger.getValue() >= 0.8) {
            robotCfg.getFlyWheels().setPower(0.5);
        }else if (driver1.right_trigger.getValue() >= 0.65) {
            robotCfg.getFlyWheels().setPower(0.4);
        }else if (driver1.right_trigger.getValue() >= 0.5) {
            robotCfg.getFlyWheels().setPower(0.3);
        }else if (driver1.right_trigger.getValue() >= 0.35) {
            robotCfg.getFlyWheels().setPower(0.2);
        }else if (driver1.right_trigger.getValue() >= 0.2) {
            robotCfg.getFlyWheels().setPower(0.1);
        } else {
            robotCfg.getFlyWheels().stop();
        }

//        robotCfg.getLiftArm().liftControlExtension(driver2.left_stick_y.getValue());

        if(driver2.y.justPressed()){
            robotCfg.getLiftArm().armExtend();
        }

        if(driver2.a.justPressed()){
            robotCfg.getLiftArm().armRetract();
        }

        if(driver2.x.justPressed()){
            if(robotCfg.getLiftArm().getWristPosition() == LiftArm.WristPositions.RIGHT){
                robotCfg.getLiftArm().wristStraight();
            }else if(robotCfg.getLiftArm().getWristPosition() == LiftArm.WristPositions.STRAIGHT){
                robotCfg.getLiftArm().wristLeft();
            }
        }

        if(driver2.b.justPressed()){
            if(robotCfg.getLiftArm().getWristPosition() == LiftArm.WristPositions.LEFT){
                robotCfg.getLiftArm().wristStraight();
            }else if(robotCfg.getLiftArm().getWristPosition() == LiftArm.WristPositions.STRAIGHT){
                robotCfg.getLiftArm().wristRight();
            }
        }
        int m = robotCfg.getMecanumControl().getMecanumMotors().getEncoder(0);
        int m1 = robotCfg.getMecanumControl().getMecanumMotors().getEncoder(1);
        int m2 = robotCfg.getMecanumControl().getMecanumMotors().getEncoder(2);
        int m3 = robotCfg.getMecanumControl().getMecanumMotors().getEncoder(3);
        telemetry.addData("motor 0 - frontRight", m);
        telemetry.addData("motor 1 - frontLeft", m1);
        telemetry.addData("motor 2 - backLeft", m2);
        telemetry.addData("motor 3 - backRight", m3);

//        telemetry.addData("lift position =", robotCfg.getLiftArm().getLiftEncoder() );
//        telemetry.addData("lift Target Position =", robotCfg.getLiftArm().getLiftTargetPosition() );





//        // Dumper control
//        if(driver2.y.isPressed()) {
//            dump.goToPreset(FutureFestRobotCfg.DumpServoPresets.OPEN);
//        } else if(driver2.a.isPressed()) {
//        } else{
//            dump.goToPreset(FutureFestRobotCfg.DumpServoPresets.OPEN);
//        }
    }

    @Override
    protected void end() {

    }
}
