package org.firstinspires.ftc.teamcode.kidFest2018;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import evlib.hardware.control.RotationControls;
import evlib.hardware.control.TranslationControls;
import evlib.opmodes.AbstractTeleOp;
import ftc.electronvolts.util.Function;
import ftc.electronvolts.util.Functions;
import ftc.electronvolts.util.InputExtractor;
import ftc.electronvolts.util.InputExtractors;
import ftc.electronvolts.util.files.Logger;
import ftc.electronvolts.util.units.Time;

/**
 * Created by ftc7393 on 9/22/2018.
 */
@TeleOp(name = "KidFest")
public class kidFestTeleOp extends AbstractTeleOp<kidFestRobotCfg> {
    private DcMotor dump = null;
    private DcMotor collector = null;
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
    protected kidFestRobotCfg createRobotCfg() {
        return new kidFestRobotCfg(hardwareMap);
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
        //noinspection SuspiciousNameCombination
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

        collector=robotCfg.getCollector();
        dump  = robotCfg.getDump();

        if(driver2.x.isPressed()) {
            driver1CollectorEnabled = false;
        } else {
            driver1CollectorEnabled = true;
        }

        // Collector logic: Driver 2 has priority, Driver 1 can be disabled as well
        if(driver2.right_bumper.isPressed() ||
                (driver1.right_bumper.isPressed() && driver1CollectorEnabled == true) ){
            collector.setPower(-0.8);
        } else if(driver2.left_bumper.isPressed()) {
            collector.setPower(0.8);
        } else {
            collector.setPower(0);
        }

        // Dumper control
        if(driver2.y.isPressed()) {
            dump.setPower(1.0);
        } else if(driver2.a.isPressed()) {
            dump.setPower(-1.0);
        } else{
            dump.setPower(0);
        }
    }

    @Override
    protected void end() {

    }
}
