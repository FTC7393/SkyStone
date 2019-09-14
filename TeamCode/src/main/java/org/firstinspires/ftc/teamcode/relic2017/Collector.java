package org.firstinspires.ftc.teamcode.relic2017;//package org.firstinspires.ftc.teamcode.relic2017;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//
//import ftc.electronvolts.util.DeadZone;
//import ftc.electronvolts.util.DigitalInputEdgeDetector;
//import ftc.electronvolts.util.Function;
//import ftc.electronvolts.util.Functions;
//import ftc.electronvolts.util.PIDController;
//import evlib.hardware.motors.Motor;
//import evlib.hardware.motors.MotorEnc;
//import evlib.hardware.sensors.DigitalSensor;
//import evlib.hardware.servos.ServoControl;
//
///**
// * Created by ftc7393 on 11/26/2017.
// * <p>
// * cannot tilt the conveyor belt while at the bottom (turn vertical or horizontal)
// * cannot open the conveyor belt while horizontal
// * cannot run conveyor belt while open
// * it cannot go too high or too low
// * cannot go more than horizontal or vertical
// */
//
//public class Collector {
//
//
//    public Collector(MotorEnc lift, MotorEnc tilt, Motor belt, ServoControl release, DigitalSensor topLimit, DigitalSensor bottomLimit, DigitalSensor tiltLimit) {
//        this.lift = lift;
//        this.tilt = tilt;
//        this.belt = belt;
//        this.release = release;
//        this.topLimit = new DigitalInputEdgeDetector(topLimit);
//        this.bottomLimit = new DigitalInputEdgeDetector(bottomLimit);
//        this.tiltLimit = new DigitalInputEdgeDetector(tiltLimit);
//        this.liftPID = new PIDController(0.03,0,0,1.0);
//    }
//
//
//    /**
//     * U=Up
//     * D=Down
//     * V=Vertical
//     * H=Horizontal
//     * 2=to
//     */
//
//    public enum Mode {//only reason public debugging in teleop
//        UP,
//        DOWN,
//        TILTUP,
//        TILTDOWN,
//        OFF,
//        UH(false),
//        UV(true),
//        DV(true),
//        DH(false),
//        UH2UV,
//        UH2UV2DV,
//        UV2UH,
//        UV2UH2DH,
//        DH2MH2DV,
//        MH2MV,
//        MV2MH,
//        DV2MV,
//        UH2DH,
//        DH2UH,
//        DH2UH2UV,
//        UV2DV,
//        DV2UV2UH,
//        DV2UV,
//        UNKNOWN,
//        UNKNOWN2UP,
//        UNKNOWN2VERTICAL,
//        UNKNOWN2BOTTOM,
//        UNKNOWN2RESET;
//
//
//        private boolean canOpenServo;
//        private boolean canUserSwitch;
//
//
//        Mode() {
//            canOpenServo = false;
//            canUserSwitch = false;
//        }
//
//        Mode(boolean haltServo) {
//            canOpenServo = haltServo;
//            canUserSwitch = true;
//        }
//
//        public boolean canOpenServo() {
//            return canOpenServo;
//        }
//
//        public boolean canUserSwitch() {
//            return canUserSwitch;
//        }
//
//
//    }
//
//    private enum ServoAction {
//        OPEN,
//        OPENING,
//        EJECT,
//        CLOSING
//    }
//
//    private enum LittleUp {
//        LITTLE_UP,
//        REACHEDLITTLE;
//
//    }
//
//    private Mode mode = Mode.UNKNOWN;//at the end of auto, it is at DH
//    private ServoAction servoAction = ServoAction.EJECT;
//    private double beltPower = 0;
//
//    private final MotorEnc lift;
//    private final MotorEnc tilt;
//    private final Motor belt;
//    private final ServoControl release;
//    private final DigitalInputEdgeDetector topLimit, bottomLimit, tiltLimit;
//    private static final double LIFT_SPEED = 1.5;//need to find value
//    private static final int LIFT_ENCODER = 260;//need to find value
//    private static final double TILT_SPEED = -0.6;//need to find value
//    private static final int TILT_ENCODER_HORIZONTAL = -275; // -300;//need to be less than this value
//    private static final int UPMID_ENCODER = 450;//need to find value
////    private static final int LITTLEUP_ENCODER_DELTA = 110;//need to find value
////    private int littleUpStartEncoder;
////    private LittleUp littleUp = LittleUp.REACHEDLITTLE;
//
//
//    private int lastLiftEncoder = 0;
//
//    // Variables for the Lift PID controller
//    private double liftSetPoint = 0;
//    private PIDController liftPID;
//    private final double LIFT_ENCODER_TOLERANCE = 10;
//    private final double LIFT_ENCODER_TOP = 1000;
//    private final double LIFT_ENCODER_MID = 450;
//    private final double LIFT_ENCODER_DV_START = -1*LIFT_ENCODER_TOP;  // Go until the bottom limit switch is pressed
//    private final double LIFT_ENCODER_DV_RELEASE = 120;
//    private final double LIFT_ENCODER_DH = 15;
//    private static final int MIN_LIFT_ENCODER_TO_RELEASE = 100;
//
//    private final double LIFT_LIMIT_FIND_POWER = 0.8;
//
//
//
//    private boolean topPressed = false, bottomPressed = false, tiltPressed = false;
//    private double liftEncoder = 0, tiltEncoder = 0;
//
//    public double getLiftEncoderValue() {
//        return liftEncoder;
//    }
//
//    public double getTiltEncoderValue() {
//        return tiltEncoder;
//    }
//
//    public boolean getTopLimit() {
//        return topPressed;
//    }
//
//    public boolean getBottomLimit() {
//        return bottomPressed;
//    }
//
//    public boolean getTiltLimit() {
//        return tiltPressed;
//    }
//
//
//    private boolean liftDone() {
//        return (liftEncoder >= liftSetPoint - LIFT_ENCODER_TOLERANCE ) &&
//                (liftEncoder <= liftSetPoint + LIFT_ENCODER_TOLERANCE);
//    }
//
//    public double getLiftSetPoint() {
//        return liftSetPoint;
//    }
//
//    public void act() {
//
//        // Update limit switches
//        topLimit.update();
//        bottomLimit.update();
//        tiltLimit.update();
//        topPressed = topLimit.isPressed();
//        bottomPressed = bottomLimit.isPressed();
//        tiltPressed = tiltLimit.isPressed();
//
//        // Always reset the lift encoder when the limit switch is just pressed
//        if (bottomLimit.justPressed()) {
//            if (liftSetPoint < liftEncoder) {
//                lift.resetEncoder();
//                liftSetPoint = liftEncoder;
//            } else {
//                lift.resetEncoder();
//            }
//        }
//
//
//        // Update the encoder positions
//        tiltEncoder = tilt.getEncoderPosition();
//        liftEncoder = lift.getEncoderPosition();
//
//        // Default is don't move the tilt
//        tilt.setSpeed(0);
//
//        // MODE STATE MACHINE
//        if (mode == Mode.UNKNOWN) {
//            // In unkown state, only pressing A is allowed
//            liftSetPoint = liftEncoder;
//        }
//        else if (mode == Mode.UH || mode == Mode.UV || mode == Mode.DH || mode == Mode.DV) {
//            // do nothing
//        }
//        else if (mode == Mode.UNKNOWN2UP) {
//            if(topPressed)
//                liftSetPoint = liftEncoder;
//            else
//                liftSetPoint = LIFT_ENCODER_TOP;
//
//            if (liftDone())
//                mode = Mode.UNKNOWN2VERTICAL;
//        }
//        else if (mode == Mode.DH2UH) {//needs to go up
//            if(topPressed)
//                liftSetPoint = liftEncoder;
//            else
//                liftSetPoint = LIFT_ENCODER_TOP;
//
//            if (liftDone())
//                mode = Mode.UH;
//        }
//        else if (mode == Mode.DH2UH2UV) {//needs to go up
//            if(topPressed)
//                liftSetPoint = liftEncoder;
//            else
//                liftSetPoint = LIFT_ENCODER_TOP;
//
//            if (liftDone())
//                mode = Mode.UH2UV;
//        }
//        else if (mode == Mode.DV2UV) {//needs to go up
//            if(topPressed)
//                liftSetPoint = liftEncoder;
//            else
//                liftSetPoint = LIFT_ENCODER_TOP;
//
//            if (liftDone())
//                mode = Mode.UV;
//        }
//        else if (mode == Mode.DV2UV2UH) {//needs to go up
//            if(topPressed)
//                liftSetPoint = liftEncoder;
//            else
//                liftSetPoint = LIFT_ENCODER_TOP;
//
//            if (liftDone())
//                mode = Mode.UV2UH;
//        }
//        else if (mode == Mode.DV2MV) {//needs to go to the middle
//            liftSetPoint = LIFT_ENCODER_MID;
//            if (liftDone())
//                mode = Mode.MV2MH;
//        }
//        else if (mode == Mode.DH2MH2DV) {//needs to go to the middle
//            liftSetPoint = LIFT_ENCODER_MID;
//            if (liftDone())
//                mode = Mode.MH2MV;
//        }
//        else if (mode == Mode.UH2DH) {//needs to go down
//            if(bottomPressed)
//                liftSetPoint = liftEncoder;
//            else
//                liftSetPoint = LIFT_ENCODER_DH;
//
//            if (liftDone())
//                mode = Mode.DH;
//        }
//        else if (mode == Mode.UV2DV) {//needs to go down
//            if(bottomPressed)
//                liftSetPoint = liftEncoder;
//            else
//                liftSetPoint = LIFT_ENCODER_DV_RELEASE;
//
//            if (liftDone())
//                mode = Mode.DV;
//        }
//        else if (mode == Mode.UNKNOWN2BOTTOM) {//needs to go down
//            if(bottomPressed)
//                mode = Mode.UNKNOWN2RESET;
//            else
//                liftSetPoint = LIFT_ENCODER_DV_START;
//
//        }
//        else if (mode == Mode.UNKNOWN2RESET) { // Go up just until the limit switch is released
//            if(bottomLimit.justReleased()) {
//                lift.resetEncoder();
//                mode = Mode.UV2DV;
//            } else {
//             //   lift.setPower(LIFT_LIMIT_FIND_POWER);  // This is done down below
//            }
//
//        }
//        else if (mode == Mode.UH2UV) {
//            tilt.setSpeed(-TILT_SPEED);
//            if (tiltPressed) {
//                tilt.resetEncoder();
//                tilt.setSpeed(0);
//                mode = Mode.UV;
//            }
//        }
//        else if (mode == Mode.UV2UH) {
//            tilt.setSpeed(TILT_SPEED);
//            if (tilt.getEncoderPosition() < TILT_ENCODER_HORIZONTAL) {
//                tilt.setSpeed(0);
//                mode = Mode.UH;
//            }
//        }
//        else if (mode == Mode.UH2UV2DV) {
//            tilt.setSpeed(-TILT_SPEED);
//            if (tiltPressed) {
//                tilt.resetEncoder();
//                tilt.setSpeed(0);
//                mode = Mode.UV2DV;
//            }
//        }
//        else if (mode == Mode.UNKNOWN2VERTICAL) {
//            tilt.setSpeed(-TILT_SPEED);
//            if (tiltPressed) {
//                tilt.resetEncoder();
//                tilt.setSpeed(0);
//                mode = Mode.UNKNOWN2BOTTOM;
//            }
//        }
//        else if (mode == Mode.UV2UH2DH) {
//            tilt.setSpeed(TILT_SPEED);
//            if (tilt.getEncoderPosition() < TILT_ENCODER_HORIZONTAL) {
//                tilt.setSpeed(0);
//                mode = Mode.UH2DH;
//            }
//        }
//        else if (mode == Mode.MH2MV) {
//            tilt.setSpeed(-TILT_SPEED);
//            if (tiltPressed) {
//                tilt.resetEncoder();
//                tilt.setSpeed(0);
//                mode = Mode.UV2DV;
//            }
//        }
//        else if (mode == Mode.MV2MH) {
//            tilt.setSpeed(TILT_SPEED);
//            if (tilt.getEncoderPosition() < TILT_ENCODER_HORIZONTAL) {
//                tilt.setSpeed(0);
//                mode = Mode.UH2DH;
//            }
//
//        }
//        else {
//            mode = Mode.UNKNOWN;
//        }
//
//
//
////        if (mode == Mode.DV) {
////            if (littleUp == LittleUp.LITTLE_UP) {
////                lift.setSpeed(LIFT_SPEED);
////                tilt.setSpeed(0);
////                if ((lift.getEncoderPosition() - littleUpStartEncoder) > LITTLEUP_ENCODER_DELTA) {
////                    lift.setSpeed(0);
////                    littleUp = LittleUp.REACHEDLITTLE;
////                }
////            }
////        }
//
//        // Update servo state machine if it is finished moving
//        if (release.isDone()) {
//            if (servoAction == ServoAction.CLOSING) {
//                servoAction = ServoAction.EJECT;
//            } else if (servoAction == ServoAction.OPENING) {
//                servoAction = ServoAction.OPEN;
//            }
//        }
//
//
//        // BELT ACTION
//        // Only run belt when servo is closed
//        if (servoAction == ServoAction.EJECT) {
//            belt.setPower(beltPower);
//        } else {
//            belt.setPower(0);
//        }
//
//
//
//        // Execute the Lift PID
//        // Don't let the target position be outside the limit switches
//        if( bottomPressed && liftSetPoint < liftEncoder ) {
//            liftSetPoint = liftEncoder;
//        }
//        else if( topPressed && liftSetPoint > liftEncoder ) {
//            liftSetPoint = liftEncoder;
//        }
//
//        double liftPower;
//        if (mode == Mode.UNKNOWN) {
//            liftPower = 0;
//        } else if (mode == Mode.UNKNOWN2RESET) {
//            liftPower = LIFT_LIMIT_FIND_POWER;
//        } else {
//            liftPower = liftPID.computeCorrection(this.liftSetPoint, this.liftEncoder);
//        }
//
//        lift.setPower(liftPower);
//
//        // Update motors
//        lift.update();
//        tilt.update();
//        belt.update();
//
//
//        lastLiftEncoder = lift.getEncoderPosition();
//
//    }
//
//
//    public void goUp() {
//        mode = Mode.UP;
//    }
//
//    public void goDown() {
//        mode = Mode.DOWN;
//    }
//
//    public void tiltUp() {
//        mode = Mode.TILTUP;
//    }
//
//    public void tiltDown() {
//        mode = Mode.TILTDOWN;
//    }
//
//    public void off() {
//        mode = Mode.OFF;
//    }
//
//    public Mode getMode() {
//        return mode;
//    }
//
////    public void doLittleUp() {
////        littleUpStartEncoder = lastLiftEncoder;
////        littleUp = LittleUp.LITTLE_UP;
////    }
//
//    public void haltServo() {
//        if (!mode.canOpenServo()) {
//            return;
//        }
//        // Can only open for UV and DV.
//        // For DV, there is another restriction:
//        //   we have to be almost to the little up value.
//        // For UV, we will always be above this anyway, unless
//        // we drift down below the limit, in which case we
//        // don't want to open anyway.
//            if (lastLiftEncoder < MIN_LIFT_ENCODER_TO_RELEASE) {
//                return;
//        }
//
//        servoAction = ServoAction.OPENING;
//        release.goToPreset(RobotCfg2017.ReleaseServoPresets.HALT, 3);
//    }
//
//    public void ejectServo() {
//        servoAction = ServoAction.CLOSING;
//        release.goToPreset(RobotCfg2017.ReleaseServoPresets.EJECT, 3);
//    }
//
//    public void toggleServo() {
//        if (servoAction == ServoAction.OPEN || servoAction == ServoAction.OPENING) {
//            ejectServo();
//        } else if (servoAction == ServoAction.EJECT || servoAction == ServoAction.CLOSING) {
//            haltServo();
//        }
//
//
//    }
//
//    public void setBeltPower(Telemetry telemetry, double frawTriggerValue, double brawTriggerValue) {
//
//        final double threshold = 0.04;
//        Function dzFunc = Functions.deadzone(new DeadZone() {
//            @Override
//            public boolean isInside(double value) {
//                return value < threshold;
//            }
//        });
//        telemetry.addData("RAW FWD: ", frawTriggerValue);
//        telemetry.addData("RAW BCK: ", brawTriggerValue);
//
//        double dzf = dzFunc.f(frawTriggerValue);
//        double dzb = dzFunc.f(brawTriggerValue);
//
//        if (dzf > threshold) {
//
//            beltPower = dzf;
//            telemetry.addData("FWD: ", dzf);
//        } else if (dzb > threshold) {
//            beltPower = -dzb;
//            telemetry.addData("BCK: ", dzb);
//        } else {
//            beltPower = 0;
//            telemetry.addData("OFF: ", beltPower);
//        }
//
//
//    }
//
//    public void unknown() {
//        if (tiltLimit.isPressed() && bottomLimit.isPressed()) {
//            // Already vertical and down, find encoder reset point
//            mode = Mode.UNKNOWN2RESET;
//        } else if (tiltLimit.isPressed()) {
//            // Already vertical, go down
//            mode = Mode.UNKNOWN2BOTTOM;
//        } else {
//            // Need to to up, vertical, then down
//            mode = Mode.UNKNOWN2UP;
//        }
//    }
//
//    public void goMH() {}
////    public void goMH() {
////        if (!mode.canUserSwitch()) {
////            return;
////        }
////        if (mode == Mode.DV) {
////            mode = Mode.DV2MV2MH;
////        } else if (mode == Mode.DH) {
////            mode = Mode.DH2MH2DV;
////        } else if (mode == Mode.UV) {
////            mode = Mode.UV2UH;
////        } else if (mode == Mode.UH) {
////            mode = mode.UH;
////        }
////    }
//
//    public void goUV() {
//        if (!mode.canUserSwitch()) {
//            return;
//        }
//        if (mode == Mode.DV) {
//            mode = Mode.DV2UV;
//        } else if (mode == Mode.DH) {
//            mode = Mode.DH2UH2UV;
//        } else if (mode == Mode.UV) {
//            mode = Mode.UV;
//        } else if (mode == Mode.UH) {
//            mode = Mode.UH2UV;
//        }
//
//    }
//
//    public void goDH() {
//        if (!mode.canUserSwitch()) {
//            return;
//        }
//        if (mode == Mode.DV) {
//            mode = Mode.DV2MV;
//        } else if (mode == Mode.DH) {
//            mode = Mode.DH;
//        } else if (mode == Mode.UV) {
//            mode = Mode.UV2UH2DH;
//        } else if (mode == Mode.UH) {
//            mode = Mode.UH2DH;
//        }
//    }
//
//    public void goDV() {
//        if (!mode.canUserSwitch()) {
//            return;
//        }
//        if (mode == Mode.DV) {
//            mode = Mode.DV;
//        } else if (mode == Mode.DH) {
//            mode = Mode.DH2MH2DV;
//        } else if (mode == Mode.UV) {
//            mode = Mode.UV2DV;
//        } else if (mode == Mode.UH) {
//            mode = Mode.UH2UV2DV;
//        }
//    }
//
//
//}
