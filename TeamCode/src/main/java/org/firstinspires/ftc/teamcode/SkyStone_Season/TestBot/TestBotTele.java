package org.firstinspires.ftc.teamcode.SkyStone_Season.TestBot;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SkyStone_Season.TeleOp.SkystoneTeleOp;

import ftc.electronvolts.util.Function;
import ftc.electronvolts.util.Functions;
import ftc.electronvolts.util.InputExtractor;
import ftc.electronvolts.util.files.Logger;
import ftc.evlib.hardware.control.RotationControls;
import ftc.evlib.hardware.control.TranslationControls;
import ftc.evlib.opmodes.AbstractTeleOp;

@TeleOp
public class TestBotTele extends AbstractTeleOp<TestBotRobotCfg> {

    DistanceSensor distanceSensor;
    ModernRoboticsI2cRangeSensor range;


    ScalingInputExtractor leftY;
    ScalingInputExtractor rightX;
    ScalingInputExtractor leftX;
    class ScalingInputExtractor implements InputExtractor<Double> {
        InputExtractor<Double> ext;
        private double factor;
        private double max;

        ScalingInputExtractor(InputExtractor<Double> ext, double f, double max) {
            this.ext = ext;
            this.factor = f;
            this.max = max;
        }
        @Override
        public Double getValue() {
            double v = ext.getValue()*factor;
            if(Math.abs(v) > max) {
                v = max * Math.signum(v);
            }
            return v;
        }
        public void setFactor(double f) {
            this.factor = f;
        }
    }
    enum MotorSpeedFactor {
        FAST(1.0), SLOW(0.4), SUPER_SLOW(0.2);
        private double factor;
        MotorSpeedFactor(double x) {
            this.factor = x;
        }
        public double getFactor() {
            return factor;
        }
    }
    private MotorSpeedFactor currentSpeedFactor = MotorSpeedFactor.FAST;

    @Override
    protected Function getJoystickScalingFunction() {
        return Functions.none();
    }

    @Override
    protected TestBotRobotCfg createRobotCfg() {
        return new TestBotRobotCfg(hardwareMap);
    }

    @Override
    protected Logger createLogger() {
        return null;
    }

    @Override
    protected void setup() {
        distanceSensor = robotCfg.getDistanceSensor();
        range = robotCfg.getRange();
    }

    @Override
    protected void setup_act() {

    }

    @Override
    protected void go() {

    }

    @Override
    protected void act() {
        telemetry.addData("distance", distanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("modern robotics distance", range.cmUltrasonic());

        double f = currentSpeedFactor.getFactor();
        // translation
        leftX = new ScalingInputExtractor(driver1.left_stick_x, -f, f);
        leftY = new ScalingInputExtractor(driver1.left_stick_y, -f, f);

        // rotation (only uses right X)
        rightX = new ScalingInputExtractor(driver1.right_stick_x, -f, f);
        robotCfg.getMecanumControl().setTranslationControl(TranslationControls.inputExtractorXY(leftY, leftX));
//        robotCfg.getMecanumControl().setRotationControl(RotationControls.teleOpGyro(leftX, robotCfg.getGyro()));
        robotCfg.getMecanumControl().setRotationControl(RotationControls.inputExtractor(rightX));


        int m0 = robotCfg.getMecanumControl().getMecanumMotors().getEncoder(0);
        int m1 = robotCfg.getMecanumControl().getMecanumMotors().getEncoder(1);
        int m2 = robotCfg.getMecanumControl().getMecanumMotors().getEncoder(2);
        int m3 = robotCfg.getMecanumControl().getMecanumMotors().getEncoder(3);
        telemetry.addData("motor 0 - frontLeft", m0);
        telemetry.addData("motor 1 - frontRight", m1);
        telemetry.addData("motor 2 - backLeft", m2);
        telemetry.addData("motor 3 - backRight", m3);


    }

    @Override
    protected void end() {

    }
}
