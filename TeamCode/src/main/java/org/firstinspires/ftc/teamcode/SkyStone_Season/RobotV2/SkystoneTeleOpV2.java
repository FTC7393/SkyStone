package org.firstinspires.ftc.teamcode.SkyStone_Season.RobotV2;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SkyStone_Season.TeleOp.AnalogInputEdgeDetector;

import ftc.electronvolts.util.Function;
import ftc.electronvolts.util.Functions;
import ftc.electronvolts.util.InputExtractor;
import ftc.electronvolts.util.files.Logger;
import ftc.electronvolts.util.units.Time;
import ftc.evlib.hardware.control.RotationControls;
import ftc.evlib.hardware.control.TranslationControls;
import ftc.evlib.opmodes.AbstractTeleOp;
//comment

/**
 * Created by ftc7393 on 9/22/2018.
 */
@TeleOp(name = "SkyStone Tele V2")

public class SkystoneTeleOpV2 extends AbstractTeleOp<SkystoneRobotCfgV2> {
    private boolean skystoneServoPresetDown = true;
    private boolean manualGrabberClosed = true;
    private AnalogInputEdgeDetector driver2RightYUp;
    private AnalogInputEdgeDetector driver2RightYDown;
    private AnalogInputEdgeDetector driver2RightXLeft;
    private AnalogInputEdgeDetector driver2RightXRight;
    private AnalogInputEdgeDetector driver2RightTrigger;
    private AnalogInputEdgeDetector driver2LeftTrigger;
    private final double liftspeed = 100;
    private final double extensionspeed = 100;
    private final double collectorspeed = 1;
    private boolean wristtoggle = false;



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
    protected SkystoneRobotCfgV2 createRobotCfg() {
        return new SkystoneRobotCfgV2(hardwareMap);
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
        this.driver2RightTrigger = new AnalogInputEdgeDetector(driver2.right_trigger,0.3, 0.7,true);
        this.driver2LeftTrigger = new AnalogInputEdgeDetector(driver2.left_trigger,0.3, 0.7,true);
}

    @Override
    protected void setup_act() {

    }
    private void forwardControl() {
        double f = currentSpeedFactor.getFactor();
        // translation
        leftX = new ScalingInputExtractor(driver1.right_stick_x, -f, f);
        leftY = new ScalingInputExtractor(driver1.left_stick_x, -f, f);

        // rotation (only uses right X)
        rightX = new ScalingInputExtractor(driver1.left_stick_y, f, f);
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
        driver2RightTrigger.update();
        driver2LeftTrigger.update();

//        telemetry.addData("backLeft",robotCfg.getMecanumControl().getMecanumMotors().getEncoder(0));
//        telemetry.addData("frontLeft",robotCfg.getMecanumControl().getMecanumMotors().getEncoder(1));
//        telemetry.addData("frontRight",robotCfg.getMecanumControl().getMecanumMotors().getEncoder(2));
//        telemetry.addData("backRight",robotCfg.getMecanumControl().getMecanumMotors().getEncoder(3));
        telemetry.addData("horizontalLimit",robotCfg.getLiftArmV2().getLowerLimitHorizontal().getValue());
        telemetry.addData("rightLimit",robotCfg.getLiftArmV2().getLowerLimitVerticalRight().getValue());
        telemetry.addData("LeftLimit",robotCfg.getLiftArmV2().getLowerLimitVerticalLeft().getValue());
        telemetry.addData("liftCommand",robotCfg.getLiftArmV2().getLiftCommand());
        telemetry.addData("extensionCommand",robotCfg.getLiftArmV2().getExtensionCommand());
        telemetry.addData("horizontalEncoder",robotCfg.getLiftArmV2().getHorizontalEncoder());
        telemetry.addData("verticalLeftEncoder",robotCfg.getLiftArmV2().getLowerLimitVerticalLeft());
        telemetry.addData("verticalRightEncoder",robotCfg.getLiftArmV2().getLowerLimitVerticalRight());

        //left stick button toggles fast and slow mode

        if(driver1.left_stick_button.justPressed()) {
            currentSpeedFactor = MotorSpeedFactor.SLOW;
        }

        if(driver1.right_stick_button.justPressed()) {
            currentSpeedFactor = MotorSpeedFactor.FAST;
        }

        forwardControl(); // driver 1 mechanum control for motors

        // Collector logic: Driver1 has control and can press right bumper for intake or left bumper for output
        // But is the right trigger is pressed in different levels (0.9, 0.6 and 0.3) it can set the collector power to different speeds


            robotCfg.getBlockCollector().setPower(-collectorspeed*(driver1.right_trigger.getValue()
            -driver1.left_trigger.getValue()));

        if (driver1.left_bumper.justPressed()){
            robotCfg.getNewFoundationMover().servosDown();
        }
        if (driver1.left_bumper.justReleased()){
            robotCfg.getNewFoundationMover().servosUp();
        }

        robotCfg.getLiftArmV2().controlLift(driver2.left_stick_y.getValue()* liftspeed);
        robotCfg.getLiftArmV2().controlExtension(driver2.right_stick_x.getValue() * extensionspeed);


        if(driver2.y.justPressed()) {
            if(wristtoggle == false) {
                wristtoggle = robotCfg.getLiftArmV2().wrist90();
            } else {
                robotCfg.getLiftArmV2().wrist0();
                wristtoggle = false;
            }
        }

        if(driver2.x.justPressed()){
//            extend horizontal slide to its outmost position
//            robotCfg.getLiftArm().armPlacingLeft();
        }

        if(driver2.b.isPressed()){
            robotCfg.getLiftArmV2().fingersRight();
        } else if(driver2.a.isPressed()){
            robotCfg.getLiftArmV2().fingersLeft();
        } else if(driver2LeftTrigger.isPressed()){
            robotCfg.getLiftArmV2().fingerEject();
        } else if(driver2RightTrigger.isPressed()){
            robotCfg.getLiftArmV2().fingersIngest();
        } else robotCfg.getLiftArmV2().fingersStop();

        if(driver2.left_bumper.justPressed()){
            robotCfg.getLiftArmV2().gripperGrab();
        }

        if(driver2.right_bumper.justPressed()){
            robotCfg.getLiftArmV2().gripperRelease();
        }

// LiftArm auto commands ///////////////////////////////////////////////////////////////////
//        if(driver2RightYDown.justPressed()){
//            robotCfg.getLiftArm().sendCommand(LiftArm.COMMANDS.STOW);
//        }

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

//        if(driver2RightXRight.justPressed()){
//            robotCfg.getLiftArm().sendCommand(LiftArm.COMMANDS.GRAB);
//        }
//
//        if (driver2.dpad_up.justPressed()){
//            robotCfg.getLiftArm().incrementPlacingLevel();
//        }
//
//        if (driver2.dpad_down.justPressed()){
//            robotCfg.getLiftArm().decrementPlacingLevel();
//        }

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

//        int m = robotCfg.getMecanumControl().getMecanumMotors().getEncoder(0);
//        int m1 = robotCfg.getMecanumControl().getMecanumMotors().getEncoder(1);
//        int m2 = robotCfg.getMecanumControl().getMecanumMotors().getEncoder(2);
//        int m3 = robotCfg.getMecanumControl().getMecanumMotors().getEncoder(3);
//        telemetry.addData("motor 0 - frontRight", m);
//        telemetry.addData("motor 1 - frontLeft", m1);
//        telemetry.addData("motor 2 - backLeft", m2);
//        telemetry.addData("motor 3 - backRight", m3);
//      telemetry.addData("extension motor", robotCfg.getLiftArm().getLift().getExtensionEncoder());
//        telemetry.addData("placing level", robotCfg.getLiftArm().getPlacingLevel());
        telemetry.addData("rightStickX", driver2.right_stick_x.getValue());
        telemetry.addData("rightStickY", driver2.right_stick_y.getValue());
        telemetry.addData("rightStickXLeft", driver2RightXLeft.getValue());
        telemetry.addData("rightStickXRight", driver2RightXRight.getValue());
        telemetry.addData("rightStickYUp", driver2RightYUp.getValue());
        telemetry.addData("rightStickYDown", driver2RightYDown.getValue());
//        telemetry.addData("Lift Arm State", robotCfg.getLiftArm().getCurrentStateName());
//        telemetry.addData("Upper Limit", robotCfg.getLiftArm().getLift().getUpperLimit());
//        telemetry.addData("Lower Limit", robotCfg.getLiftArm().getLift().getLowerLimit());
//        telemetry.addData("lift position =", robotCfg.getLiftArm().getLift().getExtensionEncoder());
//        telemetry.addData("lift Target Position =", robotCfg.getLiftArm().getLift().getExtensionSetPoint());



    }

    @Override
    protected void end() {

    }
}
