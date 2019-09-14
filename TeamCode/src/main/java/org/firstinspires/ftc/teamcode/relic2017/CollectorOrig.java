package org.firstinspires.ftc.teamcode.relic2017;//package org.firstinspires.ftc.teamcode.relic2017;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//
//import ftc.electronvolts.util.DeadZone;
//import ftc.electronvolts.util.DigitalInputEdgeDetector;
//import ftc.electronvolts.util.Function;
//import ftc.electronvolts.util.Functions;
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
//public class CollectorOrig {
//
//
//    public CollectorOrig(MotorEnc lift, MotorEnc tilt, Motor belt, ServoControl release, DigitalSensor topLimit, DigitalSensor bottomLimit, DigitalSensor tiltLimit) {
//        this.lift = lift;
//        this.tilt = tilt;
//        this.belt = belt;
//        this.release = release;
//        this.topLimit = new DigitalInputEdgeDetector(topLimit);
//        this.bottomLimit = new DigitalInputEdgeDetector(bottomLimit);
//        this.tiltLimit = new DigitalInputEdgeDetector(tiltLimit);
//
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
//        UH(true, false),
//        UV(false, true),
//        DV(false, true),
//        DH(true, false),
//        UH2UV,
//        UH2UV2DV,
//        UV2UH,
//        UV2UH2DH,
//        DH2MH,
//        MH2MV,
//        MV2MH,
//        DV2MV,
//        UH2DH,
//        DH2UH,
//        DH2UH2UV,
//        UV2DV,
//        DV2UV2UH,
//        DV2UV,
//        UNKNOWN2UP;
//
//
//        private boolean canRunBelt;
//        private boolean canOpenServo;
//        private boolean canUserSwitch;
//
//
//        Mode() {
//            canRunBelt = false;
//            canOpenServo = false;
//            canUserSwitch = false;
//        }
//
//        Mode(boolean doBelt, boolean haltServo) {
//            canRunBelt = doBelt;
//            canOpenServo = haltServo;
//            canUserSwitch = true;
//        }
//
//        public boolean canRunBelt() {
//            return canRunBelt;
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
//    private Mode mode = Mode.DV;//at the end of auto, it is at DH
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
//    private static final int TILT_ENCODER_HORIZONTAL = -270; // -300;//need to be less than this value
//    private static final int UPMID_ENCODER = 450;//need to find value
//    private static final int LITTLEUP_ENCODER_DELTA = 110;//need to find value
//    private static final int MIN_LIFT_ENCODER_TO_RELEASE = LITTLEUP_ENCODER_DELTA - 5;
//    private int littleUpStartEncoder;
//    private LittleUp littleUp = LittleUp.REACHEDLITTLE;
//    private int lastLiftEncoder = 0;
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
//    public void act() {
//
//        topLimit.update();
//        bottomLimit.update();
//        tiltLimit.update();
//        belt.update();
//
//        tiltEncoder = tilt.getEncoderPosition();
//        liftEncoder = lift.getEncoderPosition();
//        topPressed = topLimit.isPressed();
//        bottomPressed = bottomLimit.isPressed();
//        tiltPressed = tiltLimit.isPressed();
//        if (mode == Mode.UH || mode == Mode.UV || mode == Mode.DH || mode == Mode.DV) {
//            lift.setSpeed(0);
//            tilt.setSpeed(0);
//
//        } else if (mode == Mode.UNKNOWN2UP) {
//            lift.setSpeed(LIFT_SPEED);
//            tilt.setSpeed(0);
//            if (topLimit.isPressed()) {
//                lift.setSpeed(0);
//                mode = Mode.UH2UV2DV;
//            }
//        } else if (mode == Mode.DH2UH) {//needs to go up
//            lift.setSpeed(LIFT_SPEED);
//            tilt.setSpeed(0);
//
//            if (topLimit.isPressed()) {
//                lift.setSpeed(0);
//                mode = Mode.UH;
//            }
//
//
//        } else if (mode == Mode.DH2UH2UV) {//needs to go up
//            lift.setSpeed(LIFT_SPEED);
//            tilt.setSpeed(0);
//
//            if (topLimit.isPressed()) {
//                lift.setSpeed(0);
//                mode = Mode.UH2UV;
//            }
//
//        } else if (mode == Mode.DV2UV) {//needs to go up
//            lift.setSpeed(LIFT_SPEED);
//            tilt.setSpeed(0);
//
//            if (topLimit.isPressed()) {
//                lift.setSpeed(0);
//                mode = Mode.UV;
//            }
//
//        } else if (mode == Mode.DV2UV2UH) {//needs to go up
//            lift.setSpeed(LIFT_SPEED);
//            tilt.setSpeed(0);
//
//            if (topLimit.isPressed()) {
//                lift.setSpeed(0);
//                mode = Mode.UV2UH;
//            }
//
//        } else if (mode == Mode.DV2MV) {//needs to go to the middle
//            lift.setSpeed(LIFT_SPEED);
//            tilt.setSpeed(0);
//            if (lift.getEncoderPosition() > LIFT_ENCODER) {
//                lift.setSpeed(0);
//                mode = Mode.MV2MH;
//            }
//
//        } else if (mode == Mode.DH2MH) {//needs to go to the middle
//            lift.setSpeed(LIFT_SPEED);
//            tilt.setSpeed(0);
//
//            if (lift.getEncoderPosition() > LIFT_ENCODER) {
//                lift.setSpeed(0);
//                mode = Mode.MH2MV;
//            }
//        } else if (mode == Mode.UH2DH) {//needs to go down
//            lift.setSpeed(-LIFT_SPEED);
//            tilt.setSpeed(0);
//
//            if (bottomLimit.isPressed()) {
//                lift.resetEncoder();
//                lift.setSpeed(0);
//                mode = Mode.DH;
//            }
//        } else if (mode == Mode.UV2DV) {//needs to go down
//            lift.setSpeed(-LIFT_SPEED);
//            tilt.setSpeed(0);
//
//            if (bottomLimit.isPressed()) {
//                lift.resetEncoder();
//                lift.setSpeed(0);
//                mode = Mode.DV;
//            }
//        } else if (mode == Mode.UH2UV) {
//            lift.setSpeed(0);
//            tilt.setSpeed(-TILT_SPEED);
//            if (tiltLimit.isPressed()) {
//                tilt.resetEncoder();
//                tilt.setSpeed(0);
//                mode = Mode.UV;
//            }
//        } else if (mode == Mode.UV2UH) {
//            lift.setSpeed(0);
//            tilt.setSpeed(TILT_SPEED);
//            if (tilt.getEncoderPosition() < TILT_ENCODER_HORIZONTAL) {
//                tilt.setSpeed(0);
//                mode = Mode.UH;
//            }
//        } else if (mode == Mode.UH2UV2DV) {
//            lift.setSpeed(0);
//            tilt.setSpeed(-TILT_SPEED);
//            if (tiltLimit.isPressed()) {
//                tilt.resetEncoder();
//                tilt.setSpeed(0);
//                mode = Mode.UV2DV;
//            }
//        } else if (mode == Mode.UV2UH2DH) {
//            lift.setSpeed(0);
//            tilt.setSpeed(TILT_SPEED);
//            if (tilt.getEncoderPosition() < TILT_ENCODER_HORIZONTAL) {
//                tilt.setSpeed(0);
//                mode = Mode.UH2DH;
//            }
//        } else if (mode == Mode.MH2MV) {
//            lift.setSpeed(0);
//            tilt.setSpeed(-TILT_SPEED);
//            if (tiltLimit.isPressed()) {
//                tilt.resetEncoder();
//                tilt.setSpeed(0);
//                mode = Mode.UV2DV;
//            }
//        } else if (mode == Mode.MV2MH) {
//            lift.setSpeed(0);
//            tilt.setSpeed(TILT_SPEED);
//            if (tilt.getEncoderPosition() < TILT_ENCODER_HORIZONTAL) {
//                tilt.setSpeed(0);
//                mode = Mode.UH2DH;
//            }
//
//        } else {
//            mode = Mode.DV;
//        }
//
//        if (mode == Mode.DV) {
//            if (littleUp == LittleUp.LITTLE_UP) {
//                lift.setSpeed(LIFT_SPEED);
//                tilt.setSpeed(0);
//                if ((lift.getEncoderPosition() - littleUpStartEncoder) > LITTLEUP_ENCODER_DELTA) {
//                    lift.setSpeed(0);
//                    littleUp = LittleUp.REACHEDLITTLE;
//                }
//            }
//        }
//
//        if (release.isDone()) {
//            if (servoAction == ServoAction.CLOSING) {
//                servoAction = ServoAction.EJECT;
//            } else if (servoAction == ServoAction.OPENING) {
//                servoAction = ServoAction.OPEN;
//            }
//        }
//
//        // if (mode.canRunBelt) {
//        if (servoAction == ServoAction.EJECT) {
//            belt.setPower(beltPower);
//        } else {
//            belt.setPower(0);
//        }
//
//
//        tilt.update();
//        lift.update();
//
//        //code to find encoder values
////        if(mode==Mode.UP){
////            lift.setSpeed(2);
////            tilt.setSpeed(0);
////
////        }
////        else if(mode==Mode.DOWN){
////            lift.setSpeed(-2);
////            tilt.setSpeed(0);
////
////        }
////        else if(mode==Mode.TILTUP){
////            tilt.setSpeed(1);
////            lift.setSpeed(0);
////
////        }
////        else if(mode==Mode.TILTDOWN){
////            tilt.setSpeed(-1);
////            lift.setSpeed(0);
////
////        }
////        else if(mode==Mode.OFF){
////            tilt.setSpeed(0);
////            lift.setSpeed(0);
////        }
////        else{
////            mode=Mode.OFF;
////        }
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
//    public void doLittleUp() {
//        littleUpStartEncoder = lastLiftEncoder;
//        littleUp = LittleUp.LITTLE_UP;
//    }
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
//        if (!mode.canRunBelt()) {
//            beltPower = 0;
//            return;
//        }
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
//            mode = Mode.DV;
//        } else {
//            mode = Mode.UNKNOWN2UP;
//        }
//    }
//
//
//    public void goUH() {
//        if (!mode.canUserSwitch()) {
//            return;
//        }
//        if (mode == Mode.DV) {
//            mode = Mode.DV2UV2UH;
//        } else if (mode == Mode.DH) {
//            mode = Mode.DH2UH;
//        } else if (mode == Mode.UV) {
//            mode = Mode.UV2UH;
//        } else if (mode == Mode.UH) {
//            mode = mode.UH;
//        }
//    }
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
//            mode = Mode.DH2MH;
//        } else if (mode == Mode.UV) {
//            mode = Mode.UV2DV;
//        } else if (mode == Mode.UH) {
//            mode = Mode.UH2UV2DV;
//        }
//    }
//
//
//}
