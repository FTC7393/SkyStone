package org.firstinspires.ftc.teamcode.RoverRuckus;

import android.util.Log;

import com.google.common.collect.ImmutableList;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import evlib.hardware.control.RotationControls;
import evlib.hardware.control.TranslationControls;
import evlib.opmodes.AbstractTeleOp;
import evlib.util.StepTimer;
import ftc.electronvolts.util.Function;
import ftc.electronvolts.util.Functions;
import ftc.electronvolts.util.InputExtractor;
import ftc.electronvolts.util.InputExtractors;
import ftc.electronvolts.util.files.Logger;
import ftc.electronvolts.util.units.Time;

/**
 * Created by ftc7393 on 10/13/2018.
 */
@TeleOp(name = "RoverRuckusTeleOp")

public class RoverRuckusTeleOp extends AbstractTeleOp<RoverRuckusRobotCfg> {
    public Time getMatchTime() {
        return Time.fromMinutes(2000); //teleop is 2 minutes
    }

    int backFootState = 0;
    boolean latch = true;
    boolean markerOn = true;
    double rotationValue;
    double extensionValue;
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
            return ext.getValue() * factor;
        }

        public void setFactor(double f) {
            this.factor = f;
        }
    }

    private MotorSpeedFactor currentSpeedFactor = MotorSpeedFactor.SLOW;
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
    protected RoverRuckusRobotCfg createRobotCfg() {
        return new RoverRuckusRobotCfg(hardwareMap);
    }

    @Override
    protected Logger createLogger() {
        return new Logger("", "teleop.csv",
                new ImmutableList.Builder<Logger.Column>()
                        .add(
                                new Logger.Column("Rotation Joystick",
                                        new InputExtractor<Double>()
                                        {
                                            @Override
                                            public Double getValue() {
                                                return getRotationValue();
                                            }
                                        }
                                    )
                        )
                        .add(new Logger.Column("leftX", new InputExtractor<Double>() {
                            @Override
                            public Double getValue() {
                                return driver1.left_stick_x.getValue();
                            }
                        }))
                        .add(new Logger.Column("rightY", new InputExtractor<Double>() {
                            @Override
                            public Double getValue() {
                                return driver1.right_stick_y.getValue();
                            }
                        }))
                        .add(new Logger.Column("rightX", new InputExtractor<Double>() {
                            @Override
                            public Double getValue() {
                                return driver1.right_stick_x.getValue();
                            }
                        }))
                        .add(new Logger.Column("Extension Joystick", new InputExtractor<Double>() {
                            @Override
                            public Double getValue() {
                                return getExtensionValue();
                            }
                        })).addAll(robotCfg.getLoggerColumns()).build());

    }

    @Override
    protected void setup() {

    }

    @Override
    protected void setup_act() {
        robotCfg.getHanging().unlatch();
        robotCfg.getPhonePan().goToPreset(RoverRuckusRobotCfg.PhonePanPresets.MIDDLE);

    }

    private void forwardControl() {
        double f = currentSpeedFactor.getFactor();
        //rightY and leftX are inversed only if the mecanum wheels are pointed out
        rightY = new ScalingInputExtractor(InputExtractors.negative(driver1.right_stick_y), f);
        leftX = new ScalingInputExtractor(InputExtractors.negative(driver1.left_stick_x), f);
        // TODO: if mecanum wheels reversed, add InputExtractors.negative() on the driver1.right_stick_x
        rightX = new ScalingInputExtractor(InputExtractors.negative(driver1.right_stick_x), f);
        //noinspection SuspiciousNameCombination
        robotCfg.getMecanumControl().setTranslationControl(TranslationControls.inputExtractorXY(rightY, rightX));
//        robotCfg.getMecanumControl().setRotationControl(RotationControls.teleOpGyro(leftX, robotCfg.getGyro()));
        robotCfg.getMecanumControl().setRotationControl(RotationControls.inputExtractor(leftX));
    }

    @Override
    protected void go() {


    }


    StepTimer teleOpTimer = new StepTimer("teleOp", Log.INFO);

    @Override
    protected void act() {
        teleOpTimer.start();
        teleOpTimer.step("forwardControl");


        if (driver1.right_stick_button.isPressed()) {
            currentSpeedFactor = MotorSpeedFactor.SUPER_SLOW;
        } else if (driver1.left_stick_button.isPressed()) {
            currentSpeedFactor = MotorSpeedFactor.SLOW;
        }


        forwardControl();

        teleOpTimer.step("arm_control");

        // Presets:
        // Driver2 dpad UP: COLLECT ARM, Extend 1985
        // Driver2 dpad DOWN: STOW ARM, Extend 0
        // Driver2 left_bumper: SILVER DUMP, Extend 1596, Rotate 2.33
        // Driver2 right_bumper: GOLD DUMP, Extend 1985, Rotate 2.47

        // Extension presets and control\
        if(robotCfg.getArm().getLimitSwitchCount()>=1){
            if (driver2.dpad_down.isPressed()) {
                // STOW ARM
                // Move to Extension 0
                robotCfg.getArm().stowExtension();

            } else if (driver2.dpad_up.isPressed()) {
                // COLLECT ARM
                // Move to Extension 1985
                robotCfg.getArm().collectExtension();

            } else if (driver2.left_bumper.isPressed()) {
                // SILVER DUMP:
                // Move to Extension 1596
                robotCfg.getArm().dumpSilverExtension();

            } else if (driver2.right_bumper.isPressed()) {
                // GOLD DUMP:
                // Move to Extension 1985
                robotCfg.getArm().dumpGoldExtension();

            } else if (driver2.dpad_up.justReleased() || driver2.dpad_down.justReleased() ||
                    driver2.left_bumper.justReleased() || driver2.right_bumper.justReleased()) {
                robotCfg.getArm().stopExtension();
            } else {
                extensionValue = driver2.right_stick_y.getValue();
                robotCfg.getArm().controlExtension(extensionValue);
            }
        }
        else {
                extensionValue = driver2.right_stick_y.getValue();
                robotCfg.getArm().controlExtension(extensionValue);
        }

        // Rotation presets and control
        if (driver2.left_bumper.isPressed()) {
            robotCfg.getArm().dumpSilverRotation();

        } else if (driver2.right_bumper.isPressed()) {
            robotCfg.getArm().dumpGoldRotation();

        } else if (driver2.right_bumper.justReleased() || driver2.left_bumper.justReleased()) {
            robotCfg.getArm().stopRotation();

        } else {
            rotationValue = driver2.left_stick_y.getValue();
            robotCfg.getArm().controlRotation(rotationValue);
        }

        teleOpTimer.step("collector_control");


        //if(driver2.right_bumper.isPressed() && !driver2.left_bumper.isPressed()){
        //    robotCfg.getCollector().leftDoor();
        //}
        //else if(driver2.left_bumper.isPressed() && !driver2.right_bumper.isPressed()) {
        //    robotCfg.getCollector().rightDoor();
        //}
        //else
        if (driver1.right_bumper.isPressed()) {
            robotCfg.getCollector().openLeftDoor();
        } else {
            robotCfg.getCollector().closeLeftDoor();
        }
        if (driver1.left_bumper.isPressed()) {
            robotCfg.getCollector().openRightDoor();
        } else {
            robotCfg.getCollector().closeRightDoor();
        }

        double collectorValue = driver1.left_trigger.getValue() + (-driver1.right_trigger.getValue())
                + driver2.left_trigger.getValue() + (-driver2.right_trigger.getValue());
        robotCfg.getCollector().collectionPower(collectorValue);


        teleOpTimer.step("backFoot_control");


        if (driver1.y.justPressed() || driver2.y.justPressed()) {
            backFootState = 0;  // Commanded to go to locked (starting) position
        } else if (driver1.x.justPressed() || driver2.x.justPressed()) {
            if (backFootState == 0 || backFootState == 1) {
                backFootState = 2;  // Opened (above ground)
            } else if (backFootState == 2) {
                backFootState = 1;  // Deployed (touching ground)
            }
        }
        if (robotCfg.getArm().getPotentiometerValue() > 1.7) {
            // Arm is up, we can't go to position 0
            if (backFootState == 0) {
                backFootState = 1;
            }
        }

        if (backFootState == 0) {
            robotCfg.getBackFoot().goToPreset(RoverRuckusRobotCfg.BackFootPresets.LOCKED);
        } else if (backFootState == 1) {
            robotCfg.getBackFoot().goToPreset(RoverRuckusRobotCfg.BackFootPresets.OPENED);
        } else if (backFootState == 2) {
            robotCfg.getBackFoot().goToPreset(RoverRuckusRobotCfg.BackFootPresets.DEPLOYED);
        }


//        telemetry.addData("upLimit",robotCfg.getHanging().latchLimit.getValue());
//        telemetry.addData("downLimit",robotCfg.getHanging().unlatchLimit.getValue());

        teleOpTimer.step("Hanging/Latch/Marker");

        if (driver1.dpad_up.isPressed()) {
            robotCfg.getHanging().upHanging();
        } else if (driver1.dpad_down.isPressed()) {

            robotCfg.getHanging().downHanging();
        } else {
            robotCfg.getHanging().stopHanging();

        }


        if (driver1.b.justPressed()) {
            if (latch == true) {
                //telemetry.addLine("latch");

                robotCfg.getHanging().latch();
                latch = false;

            } else if (latch == false) {
                //telemetry.addLine("unlatch");

                robotCfg.getHanging().unlatch();
                latch = true;
            }
        }


        if (driver2.right_stick_button.justPressed()) {
            if (markerOn == true) {
                robotCfg.getMarker().goToPreset(RoverRuckusRobotCfg.MarkerPresets.RELEASE);
                markerOn = false;
            } else if (markerOn == false) {
                robotCfg.getMarker().goToPreset(RoverRuckusRobotCfg.MarkerPresets.HOLD);
                markerOn = true;

            }
        }


        teleOpTimer.step("telemetry");

//        telemetry.addData("VelocityR",robotCfg.getMecanumControl().getVelocityR());
//
//        telemetry.addData("VelocityX",robotCfg.getMecanumControl().getVelocityX());
//        telemetry.addData("VelocityY",robotCfg.getMecanumControl().getVelocityY());
//
//        telemetry.addData("rotationJoystick",driver1.right_stick_y.getValue());

//
//        telemetry.addData("leftX",driver1.left_stick_x.getValue());
//        telemetry.addData("rightX",driver1.right_stick_x.getValue());
//        telemetry.addData("rightY",driver1.right_stick_y.getValue());
//        telemetry.addData("leftX",driver1.left_stick_x.getValue());
//        telemetry.addData("rightX",driver1.right_stick_x.getValue());
//        telemetry.addData("scalingRightY",rightY.getValue());
//        telemetry.addData("scalingLeftX",leftX.getValue());
////
//        telemetry.addData("scalingRightX",rightX.getValue());
//testARm

//        telemetry.addData("rotationJoystick",rotationValue);

        telemetry.addData("rotationSetPoint", robotCfg.getArm().getRotationSetPoint());
//        telemetry.addData("a",robotCfg.getArm().getExponential());
        telemetry.addData("rotationEncoder", robotCfg.getArm().getRotationEncoder());
        telemetry.addData("extensionEncoder", robotCfg.getArm().getExtensionEncoder());
        telemetry.addData("potentiomenterValue", robotCfg.getArm().getPotentiometerValue());

//        telemetry.addData("frontLeft",robotCfg.frontLeft.getEncoderPosition());
//        telemetry.addData("frontRight",robotCfg.frontRight.getEncoderPosition());
//        telemetry.addData("backLeft",robotCfg.backLeft.getEncoderPosition());
//        telemetry.addData("backRight",robotCfg.backRight.getEncoderPosition());


//        telemetry.addData("rotationMaxValue",robotCfg.getArm().rotationMaxPosition);


        teleOpTimer.stop();

    }

    @Override
    protected void end() {

    }

    double getRotationValue() {
        return rotationValue;
    }

    double getExtensionValue() {
        return extensionValue;
    }

}
