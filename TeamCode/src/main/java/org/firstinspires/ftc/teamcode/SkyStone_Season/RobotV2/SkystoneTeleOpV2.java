package org.firstinspires.ftc.teamcode.SkyStone_Season.RobotV2;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SkyStone_Season.TeleOp.AnalogInputEdgeDetector;

import ftc.electronvolts.util.Function;
import ftc.electronvolts.util.Functions;
import ftc.electronvolts.util.InputExtractor;
import ftc.electronvolts.util.files.Logger;
import ftc.electronvolts.util.units.Time;
import ftc.evlib.hardware.control.RotationControls;
import ftc.evlib.hardware.control.TranslationControls;
import ftc.evlib.opmodes.AbstractTeleOp;
import ftc.evlib.util.ImmutableList;


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
    private final double extensionspeed = 200;
    private final double collectorspeed = 1;
    private boolean wristtoggle = false;
    private boolean isLeftActive = false; //is the user actively trying to change the lift?
    private boolean isRightActive = false; //same as above but for extension

    //    private  AnalogSensor cycleTime;
    private enum FoundationMoverPosition{
        UP,
        READY,
        DOWN
    }
    private FoundationMoverPosition foundationMoverPosition = FoundationMoverPosition.UP;



    @Override
    public Time getMatchTime() {
        return Time.fromMinutes(200); //teleop is 2 minutes
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
        FAST(1.0), SLOW(0.1), SUPER_SLOW(0.2);
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
        Logger l = new Logger("log", ".csv", ImmutableList.of(
                new Logger.Column("vertical slide left encoder", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return robotCfg.getLiftArmV2().getVerticalLeftEncoder();
                    }
                }), new Logger.Column("vertical slide right encoder", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return robotCfg.getLiftArmV2().getVerticalRightEncoder();
                    }
                }), new Logger.Column("horizontal slide encoder", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return robotCfg.getLiftArmV2().getHorizontalEncoder();
                    }
                }), new Logger.Column("left vertical limit switch", new InputExtractor<Boolean>() {
                    @Override
                    public Boolean getValue() {
                        return robotCfg.getLiftArmV2().getLowerLimitVerticalLeft().getValue();
                    }
                }), new Logger.Column("right vertical limit switch", new InputExtractor<Boolean>() {
                    @Override
                    public Boolean getValue() {
                        return robotCfg.getLiftArmV2().getLowerLimitVerticalRight().getValue();
                    }
                }), new Logger.Column("horizontal limit switch", new InputExtractor<Boolean>() {
                    @Override
                    public Boolean getValue() {
                        return robotCfg.getLiftArmV2().getLowerLimitHorizontal().getValue();
                    }
                }), new Logger.Column("lift command - liftArmV2", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return robotCfg.getLiftArmV2().getLiftCommand();
                    }
                }), new Logger.Column("lift command linear slide - left", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return robotCfg.getLiftArmV2().getVerticalSlideLeft().getExtensionSetPoint();
                    }
                }), new Logger.Column("lift command linear slide - right", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return robotCfg.getLiftArmV2().getVerticalSlideRight().getExtensionSetPoint();
                    }
                }), new Logger.Column("extension command", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return robotCfg.getLiftArmV2().getExtensionCommand();
                    }
                })
        ));
        return l;
    }

    @Override
    protected void setup() {
        this.driver2RightYDown = new AnalogInputEdgeDetector(driver2.right_stick_y, 0.3, 0.7,false);
        this.driver2RightYUp = new AnalogInputEdgeDetector(driver2.right_stick_y,0.3, 0.7,true);
        this.driver2RightXRight = new AnalogInputEdgeDetector(driver2.right_stick_x,0.3, 0.7,false);
        this.driver2RightXLeft = new AnalogInputEdgeDetector(driver2.right_stick_x,0.3, 0.7,true);
        this.driver2RightTrigger = new AnalogInputEdgeDetector(driver2.right_trigger,0.3, 0.7,true);
        this.driver2LeftTrigger = new AnalogInputEdgeDetector(driver2.left_trigger,0.3, 0.7,true);
        robotCfg.getOdometryServo().goToPreset(SkystoneRobotCfgV2.OdometryServoPresets.DOWN);
//        cycleTime = new AnalogSensor() {
//            long timeAtLastRead = 0;
//            @Override
//            public Double getValue() {
//                long newSystemTime = System.currentTimeMillis();
//                long delta = newSystemTime - timeAtLastRead;
//                timeAtLastRead = newSystemTime;
//                return delta + 0.0;
//            }
//        };
}

    @Override
    protected void setup_act() {
        robotCfg.getOdometryServo().act();
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


        int m = robotCfg.getMecanumControl().getMecanumMotors().getEncoder(0);
        int m1 = robotCfg.getMecanumControl().getMecanumMotors().getEncoder(1);
        int m2 = robotCfg.getMecanumControl().getMecanumMotors().getEncoder(2);
        int m3 = robotCfg.getMecanumControl().getMecanumMotors().getEncoder(3);
        telemetry.addData("motor 0 ", m);
        telemetry.addData("motor 1 ", m1);
        telemetry.addData("motor 2 ", m2);
        telemetry.addData("motor 3 ", m3);

        driver2RightYDown.update();
        driver2RightYUp.update();
        driver2RightXRight.update();
        driver2RightXLeft.update();
        driver2RightTrigger.update();
        driver2LeftTrigger.update();

//        telemetry.addData("horizontalLimit",robotCfg.getLiftArmV2().getLowerLimitHorizontal().getValue());
//        telemetry.addData("rightLimit",robotCfg.getLiftArmV2().getLowerLimitVerticalRight().getValue());
//        telemetry.addData("LeftLimit",robotCfg.getLiftArmV2().getLowerLimitVerticalLeft().getValue());
//        telemetry.addData("liftCommand",robotCfg.getLiftArmV2().getLiftCommand());
//        telemetry.addData("extensionCommand",robotCfg.getLiftArmV2().getExtensionCommand());
//        telemetry.addData("horizontalEncoder",robotCfg.getLiftArmV2().getHorizontalEncoder());
        telemetry.addData("verticalLeftEncoder",robotCfg.getLiftArmV2().getVerticalLeftEncoder());
        telemetry.addData("verticalRightEncoder",robotCfg.getLiftArmV2().getVerticalRightEncoder());
//        telemetry.addData("driver 2 left trigger", driver2.left_trigger.getValue());
//        telemetry.addData("driver 2 right trigger", driver2.right_trigger.getValue());
        telemetry.addData("internal block detector", robotCfg.getBlockDetector().getDistance(DistanceUnit.CM));
        telemetry.addData("plusY distance sensor", robotCfg.getPlusYDistanceSensor().getDistance(DistanceUnit.CM));
        telemetry.addData("minusY distance sensor", robotCfg.getMinusYDistanceSensor().getDistance(DistanceUnit.CM));
        telemetry.addData("minusX distance sensor", robotCfg.getMinusXDistanceSensor().getDistance(DistanceUnit.CM));
        telemetry.addData("odometry Encoder", robotCfg.getOdometryWheelSensor().getValue());

        //left stick button toggles fast and slow mode

        if(driver1.left_stick_button.justPressed()) {
            currentSpeedFactor = MotorSpeedFactor.SLOW;
        }

        if(driver1.right_stick_button.justPressed()) {
            currentSpeedFactor = MotorSpeedFactor.FAST;
        }
        boolean up = driver2.dpad_up.isPressed();
        boolean down = driver2.dpad_down.isPressed();
        boolean right = driver2.dpad_right.isPressed();
        boolean left = driver2.dpad_left.isPressed();
        //There are two ways to control the fingers, the priority way is to
        //control using the left and right tirggers as well as the a and b buttons,
        //the alternate way of controlling the fingers is through the dpad.

        if(driver2.right_trigger.getValue()>0.5){
            robotCfg.getLiftArmV2().fingersIngest();
        } else if(driver2.left_trigger.getValue()>0.5) {
            robotCfg.getLiftArmV2().fingerEject();
        }else if(driver2.a.isPressed()){
            robotCfg.getLiftArmV2().fingersRight();
        } else if(driver2.b.isPressed()){
            robotCfg.getLiftArmV2().fingersLeft();
        } else if(up) {
            robotCfg.getLiftArmV2().fingersIngest();
        } else if(down) {
            robotCfg.getLiftArmV2().fingerEject();
        } else if(right) {
            robotCfg.getLiftArmV2().fingersRight();
        } else if(left) {
            robotCfg.getLiftArmV2().fingersLeft();
        } else {
            robotCfg.getLiftArmV2().fingersStop();
        }

        forwardControl(); // driver 1 mechanum control for motors

        // Collector logic: Driver1 has control and can press right bumper for intake or left bumper for output
        // But is the right trigger is pressed in different levels (0.9, 0.6 and 0.3) it can set the collector power to different speeds


            robotCfg.getBlockCollector().setPower(-collectorspeed*(driver1.right_trigger.getValue()
            -driver1.left_trigger.getValue()));

        if (driver1.right_bumper.justPressed()){
            if( foundationMoverPosition == FoundationMoverPosition.UP) {
                foundationMoverPosition = FoundationMoverPosition.DOWN;
                robotCfg.getNewFoundationMover().servosDown();
            } else if (foundationMoverPosition == FoundationMoverPosition.DOWN){
                foundationMoverPosition = FoundationMoverPosition.UP;
                robotCfg.getNewFoundationMover().servosUp();
            } else if (foundationMoverPosition == FoundationMoverPosition.READY) {
                foundationMoverPosition = FoundationMoverPosition.UP;
                robotCfg.getNewFoundationMover().servosUp();
            }else {
                throw new RuntimeException("Forgot to deal with additional state/states");
            }
        }

        if (driver1.left_bumper.justPressed()){
            if( foundationMoverPosition == FoundationMoverPosition.UP) {
                foundationMoverPosition = FoundationMoverPosition.READY;
                robotCfg.getNewFoundationMover().servosReady();
            } else if (foundationMoverPosition == FoundationMoverPosition.DOWN){
                foundationMoverPosition = FoundationMoverPosition.READY;
                robotCfg.getNewFoundationMover().servosReady();
            } else if (foundationMoverPosition == FoundationMoverPosition.READY) {
                foundationMoverPosition = FoundationMoverPosition.DOWN;
                robotCfg.getNewFoundationMover().servosDown();
            }else {
                throw new RuntimeException("Forgot to deal with additional state/states");
            }
        }
         double d2_left_y = -driver2.left_stick_y.getValue();
         double d2_right_y = -driver2.right_stick_y.getValue();

        if(d2_left_y != 0) {
            isLeftActive = true;
            robotCfg.getLiftArmV2().controlLift(d2_left_y*liftspeed);
        } else {
            if(isLeftActive) {
                robotCfg.getLiftArmV2().freezeLift();
                isLeftActive = false;
            }
        }

        if(d2_right_y != 0) {
            isRightActive = true;
            robotCfg.getLiftArmV2().controlExtension(d2_right_y * extensionspeed);
        } else {
            if(isRightActive) {
                robotCfg.getLiftArmV2().freezeExtension();
                isRightActive = false;
            }
        }


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



        if(driver2.right_bumper.isPressed()){
            robotCfg.getLiftArmV2().gripperRelease();
        }else {
            robotCfg.getLiftArmV2().gripperGrab();
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

//        telemetry.addData("rightStickX", driver2.right_stick_x.getValue());
//        telemetry.addData("rightStickY", driver2.right_stick_y.getValue());
//        telemetry.addData("rightStickXLeft", driver2RightXLeft.getValue());
//        telemetry.addData("rightStickXRight", driver2RightXRight.getValue());
//        telemetry.addData("rightStickYUp", driver2RightYUp.getValue());
//        telemetry.addData("rightStickYDown", driver2RightYDown.getValue());



    }

    @Override
    protected void end() {

    }
}
