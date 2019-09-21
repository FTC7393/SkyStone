package org.firstinspires.ftc.teamcode.skystone2019.ri3d;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.futurefest2019.FutureFestRobotCfg;

import ftc.electronvolts.util.Function;
import ftc.electronvolts.util.Functions;
import ftc.electronvolts.util.InputExtractor;
import ftc.electronvolts.util.InputExtractors;
import ftc.electronvolts.util.files.Logger;
import ftc.electronvolts.util.units.Time;
import ftc.evlib.hardware.control.RotationControls;
import ftc.evlib.hardware.control.TranslationControls;
import ftc.evlib.hardware.servos.ServoControl;
import ftc.evlib.hardware.servos.Servos;
import ftc.evlib.opmodes.AbstractTeleOp;

/**
 * Created by ftc7393 on 9/22/2018.
 */
@TeleOp(name = "Ri3D Teleop")
public class Ri3DjdvTeleOp extends AbstractTeleOp<Ri3DjdvRobotCfg> {
    private ServoControl dump = null;
    private DcMotor lift = null;
    boolean driver1CollectorEnabled = true;
    private ServoControl rotateServo;
    private ServoControl spinServo;
    private ServoControl grabServo;


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
    protected Ri3DjdvRobotCfg createRobotCfg() {
        return new Ri3DjdvRobotCfg(hardwareMap);
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
        rotateServo = robotCfg.getServo(Ri3DjdvRobotCfg.Ri3DServoEnum.ROTATE_SERVO);
        spinServo = robotCfg.getServo(Ri3DjdvRobotCfg.Ri3DServoEnum.SPIN_SERVO);
        grabServo = robotCfg.getServo(Ri3DjdvRobotCfg.Ri3DServoEnum.GRAB_SERVO);

    }


    private void forwardControl() {
        double f = currentSpeedFactor.getFactor();
        rightY = new ScalingInputExtractor(driver1.right_stick_y, f);
        leftX = new ScalingInputExtractor(driver1.right_stick_x, f);
        rightX = new ScalingInputExtractor(InputExtractors.negative(driver1.left_stick_x), -1.0*f);
        //noinspection SuspiciousNameCombination
        robotCfg.getMecanumControl().setTranslationControl(TranslationControls.inputExtractorXY(rightY, rightX));
//        robotCfg.getMecanumControl().setRotationControl(RotationControls.teleOpGyro(leftX, robotCfg.getGyro()));
        robotCfg.getMecanumControl().setRotationControl(RotationControls.inputExtractor(leftX));
    }
    @Override
    protected void go() {


    }

boolean rotateToggle = true;
    boolean grabToggle = false;

    @Override
    protected void act() {
        forwardControl();

        lift =robotCfg.getLift();


        if(driver1.dpad_up.isPressed()){
            lift.setPower(-1);
        } else if(driver1.dpad_down.isPressed()){
            lift.setPower(1);
        } else{
            lift.setPower(0);
        }




        // Grab control
        if(driver1.right_bumper.justPressed() || driver1.left_bumper.isPressed()){
            grabToggle=!grabToggle;
        }

        if(grabToggle){
            grabServo.goToPreset(Ri3DjdvRobotCfg.GrabServoPresets.CLOSED);
        } else{
            grabServo.goToPreset(Ri3DjdvRobotCfg.GrabServoPresets.OPEN);

        }

        // Spin control
        if(driver1.a.justPressed()) {
            spinServo.goToPreset(Ri3DjdvRobotCfg.SpinServoPresets.CENTER);

        } else if(driver1.b.justPressed()){
            spinServo.goToPreset(Ri3DjdvRobotCfg.SpinServoPresets.RIGHT);

        }else if(driver1.x.justPressed()){
            spinServo.goToPreset(Ri3DjdvRobotCfg.SpinServoPresets.LEFT);
        }

        // Rotate control

        if(driver1.y.justPressed()){
            rotateToggle=!rotateToggle;
        }

        if(rotateToggle){
            rotateServo.goToPreset(Ri3DjdvRobotCfg.RotateServoPresets.GRAB);

        }else{
            rotateServo.goToPreset(Ri3DjdvRobotCfg.RotateServoPresets.DROP);

        }

    }

    @Override
    protected void end() {

    }
}
