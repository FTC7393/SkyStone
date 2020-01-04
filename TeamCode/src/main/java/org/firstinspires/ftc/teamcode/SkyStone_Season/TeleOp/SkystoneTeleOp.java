package org.firstinspires.ftc.teamcode.SkyStone_Season.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.SkyStone_Season.SkystoneRobotCfg;

import ftc.electronvolts.util.Function;
import ftc.electronvolts.util.Functions;
import ftc.electronvolts.util.InputExtractor;
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
    private boolean skystoneServoPresetDown = true;
    private boolean manualGrabberClosed = true;
    private AnalogInputEdgeDetector driver2RightYUp;
    private AnalogInputEdgeDetector driver2RightYDown;
    private AnalogInputEdgeDetector driver2RightXLeft;
    private AnalogInputEdgeDetector driver2RightXRight;

    @Override
    public Time getMatchTime() {
        return Time.fromMinutes(2); //teleop is 2 minutes
    }


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
    private MotorSpeedFactor currentSpeedFactor = MotorSpeedFactor.FAST;
    private MotorSpeedFactor lastXSpeedFactor = currentSpeedFactor;

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
        this.driver2RightYDown = new AnalogInputEdgeDetector(driver2.right_stick_y, 0.3, 0.7,false);
        this.driver2RightYUp = new AnalogInputEdgeDetector(driver2.right_stick_y,0.3, 0.7,true);
        this.driver2RightXRight = new AnalogInputEdgeDetector(driver2.right_stick_x,0.3, 0.7,false);
        this.driver2RightXLeft = new AnalogInputEdgeDetector(driver2.right_stick_x,0.3, 0.7,true);
    }

    @Override
    protected void setup_act() {

    }
    private void forwardControl() {
        double f = currentSpeedFactor.getFactor();
        // translation
        leftX = new ScalingInputExtractor(driver1.left_stick_x, -f, f);
        leftY = new ScalingInputExtractor(driver1.left_stick_y, -f, f);

        // rotation (only uses right X)
        rightX = new ScalingInputExtractor(driver1.right_stick_x, -f, f);
        robotCfg.getMecanumControl().setTranslationControl(TranslationControls.inputExtractorXY(leftY, leftX));
//        robotCfg.getMecanumControl().setRotationControl(RotationControls.teleOpGyro(leftX, robotCfg.getGyro()));
        robotCfg.getMecanumControl().setRotationControl(RotationControls.inputExtractor(rightX));
    }
    @Override
    protected void go() {


    }

    @Override
    protected void act() {

        driver2RightYDown.update();
        driver2RightYUp.update();
        driver2RightXRight.update();
        driver2RightXLeft.update();

        //left stick button toggles fast and slow mode

        if(driver1.left_stick_button.justPressed() || driver1.right_stick_button.justPressed()) {
            if(currentSpeedFactor == MotorSpeedFactor.FAST) {
                currentSpeedFactor = MotorSpeedFactor.SLOW;
            } else {
                currentSpeedFactor = MotorSpeedFactor.FAST;
            }
        }

        forwardControl(); // driver 1 mechanum control for motors

        // Collector logic: Driver1 has control and can press right bumper for intake or left bumper for output
        // But is the right trigger is pressed in different levels (0.9, 0.6 and 0.3) it can set the collector power to different speeds


            robotCfg.getFlyWheels().setPower(0.45*(driver1.right_trigger.getValue()+driver2.right_trigger.getValue()
            -driver1.left_trigger.getValue()-driver2.left_trigger.getValue()));

        robotCfg.getLiftArm().getLift().controlExtension(driver2.left_stick_y.getValue());

        if(driver2.y.justPressed()){
            robotCfg.getLiftArm().armPlacing();
        }

        if(driver2.a.justPressed()){
            robotCfg.getLiftArm().armGrabbing();
        }

        if(driver2.x.justPressed()){
            robotCfg.getLiftArm().armPlacingLeft();
        }

        if(driver2.b.justPressed()){
            robotCfg.getLiftArm().armStowed();
        }

        if(driver2.right_bumper.justPressed()){
            if(manualGrabberClosed) {
                manualGrabberClosed = false;
                robotCfg.getLiftArm().release();
            } else {
                manualGrabberClosed = true;
                robotCfg.getLiftArm().grab();
            }
        }

// LiftArm auto commands ///////////////////////////////////////////////////////////////////
        if(driver2RightYDown.justPressed()){
            robotCfg.getLiftArm().sendCommand(LiftArm.COMMANDS.STOW);
        }

//        if(driver2RightYUp.justPressed()){
//            robotCfg.getLiftArm().sendCommand(LiftArm.COMMANDS.PLACE);
//        }
//
//        if(driver2RightXLeft.justPressed()){
//            robotCfg.getLiftArm().sendCommand(LiftArm.COMMANDS.DROP);
//        }
//
//        if(driver2RightXLeft.justReleased()){
//            robotCfg.getLiftArm().sendCommand(LiftArm.COMMANDS.UNDROP);
//        }

        if(driver2RightXRight.justPressed()){
            robotCfg.getLiftArm().sendCommand(LiftArm.COMMANDS.GRAB);
        }

        if (driver2.dpad_up.justPressed()){
            robotCfg.getLiftArm().incrementPlacingLevel();
        }

        if (driver2.dpad_down.justPressed()){
            robotCfg.getLiftArm().decrementPlacingLevel();
        }

////////////////////////////////////////////////////////////////////////////////////////////
      /**  if (driver1.dpad_left.justPressed()) {
            skystoneServoPresetDown = !skystoneServoPresetDown;
            if(skystoneServoPresetDown) {
                robotCfg.getStoneScraperServo().goToPreset(SkystoneRobotCfg.StoneScraperServoPresets.DOWN);
            } else {
                robotCfg.getStoneScraperServo().goToPreset(SkystoneRobotCfg.StoneScraperServoPresets.UP);
            }
        }
*/
        if ((driver1.left_bumper.justPressed() && !driver2.left_bumper.isPressed()) ||
                (driver2.left_bumper.justPressed() && !driver1.left_bumper.isPressed())){
            robotCfg.getFoundationMover().servosDown();
        }
        if ((driver1.left_bumper.justReleased() && !driver2.left_bumper.isPressed()) ||
                (driver2.left_bumper.justReleased() && !driver1.left_bumper.isPressed())){
            robotCfg.getFoundationMover().servosUp();
        }


//        int m = robotCfg.getMecanumControl().getMecanumMotors().getEncoder(0);
//        int m1 = robotCfg.getMecanumControl().getMecanumMotors().getEncoder(1);
//        int m2 = robotCfg.getMecanumControl().getMecanumMotors().getEncoder(2);
//        int m3 = robotCfg.getMecanumControl().getMecanumMotors().getEncoder(3);
//        telemetry.addData("motor 0 - frontRight", m);
//        telemetry.addData("motor 1 - frontLeft", m1);
//        telemetry.addData("motor 2 - backLeft", m2);
//        telemetry.addData("motor 3 - backRight", m3);
//      telemetry.addData("extension motor", robotCfg.getLiftArm().getLift().getExtensionEncoder());
        telemetry.addData("placing level", robotCfg.getLiftArm().getPlacingLevel());
        telemetry.addData("rightStickX", driver2.right_stick_x.getValue());
        telemetry.addData("rightStickY", driver2.right_stick_y.getValue());
        telemetry.addData("rightStickXLeft", driver2RightXLeft.getValue());
        telemetry.addData("rightStickXRight", driver2RightXRight.getValue());
        telemetry.addData("rightStickYUp", driver2RightYUp.getValue());
        telemetry.addData("rightStickYDown", driver2RightYDown.getValue());
        telemetry.addData("Lift Arm State", robotCfg.getLiftArm().getCurrentStateName());
        telemetry.addData("Upper Limit", robotCfg.getLiftArm().getLift().getUpperLimit());
        telemetry.addData("Lower Limit", robotCfg.getLiftArm().getLift().getLowerLimit());
        telemetry.addData("lift position =", robotCfg.getLiftArm().getLift().getExtensionEncoder());
        telemetry.addData("lift Target Position =", robotCfg.getLiftArm().getLift().getExtensionSetPoint());



    }

    @Override
    protected void end() {

    }
}
