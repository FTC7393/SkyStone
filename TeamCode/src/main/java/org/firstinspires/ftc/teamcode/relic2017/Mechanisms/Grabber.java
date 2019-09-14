package org.firstinspires.ftc.teamcode.relic2017.Mechanisms;

import android.util.Log;

import org.firstinspires.ftc.teamcode.relic2017.Sparky2017.RobotCfg2017;

import ftc.evlib.hardware.motors.MotorEnc;
import ftc.evlib.hardware.sensors.DigitalSensor;
import ftc.evlib.hardware.servos.ServoControl;
import ftc.evlib.util.StepTimer;
import ftc.electronvolts.util.DigitalInputEdgeDetector;
import ftc.electronvolts.util.PIDController;

/**
 * Created by ftc7393 on 12/30/2017.
 */

public class Grabber {

    public Grabber(MotorEnc lift, MotorEnc flip, DigitalSensor topLimit, DigitalSensor bottomLimit, ServoControl topLeftRelease, ServoControl topRightRelease, ServoControl bottomLeftRelease, ServoControl bottomRightRelease, DigitalSensor flipLimit) {
        this.lift = lift;
        this.flip = flip;
        this.topLeftRelease = topLeftRelease;
        this.topRightRelease = topRightRelease;
        this.bottomRightRelease = bottomRightRelease;
        this.bottomLeftRelease = bottomLeftRelease;
        this.topLimit = new DigitalInputEdgeDetector(topLimit);
        this.bottomLimit = new DigitalInputEdgeDetector(bottomLimit);
        this.flipLimit = new DigitalInputEdgeDetector(flipLimit);
//        this.liftPID = new PIDController(0.003,0,0,1.0);
//        this.flipPID = new PIDController(0.005,0,.1,.5);



    }





    public enum Mode {//only reason public debugging in teleop
        UP,//release at top
        GO_UP,
        DOWN,//grab or release at bottom
        GO_DOWN,
        MIDDLE,
        GO_MIDDLE,
        GO_LITTLE_UP,
        LITTLE_UP,
        END_GAME
    }

//    private enum ServoAction {
//        HALT_ALL,
//        GRAB_All,
//        EJECT_All,
//        TOP_ONLY_GRAB,
//        TOP_ONLY_EJECT,
//        BOTTOM_ONLY_EJECT,
//        BOTTOM_ONLY_GRAB,
//        TOP_LEFT,
//        TOP_RIGHT,
//        BOTTOM_LEFT,
//        BOTTOM_RIGHT
//    }

    private enum FlipAction {
        UP,
        DOWN
    }




    private Mode mode = Mode.DOWN;//at the end of auto, it is at DH
//    private ServoAction servoAction = ServoAction.HALT_ALL;
//    private ServoAction lastServoAction = null;
    private FlipAction flipAction = FlipAction.DOWN;


    private final MotorEnc lift;
    private final MotorEnc flip;

    private final ServoControl topLeftRelease;
    private final ServoControl topRightRelease;
    private final ServoControl bottomLeftRelease;
    private final ServoControl bottomRightRelease;


    private final DigitalInputEdgeDetector topLimit, bottomLimit,flipLimit;
    private PIDController liftPID;
//    private PIDController flipPID;

    private static final double LIFT_SPEED = 1.5;//need to find value
    private  double liftSpeed = 0;//need to find value

    private static final double DOWN_LIFT_SPEED = .25;
    private static final double SERVO_SPEED=2;
    private static final double LEFT_SERVO_SPEED=.5;

    private static final int LIFT_ENCODER = 260;//need to find value

    private static final int UPMID_ENCODER = 800;
    private static final int LITTLE_UP = 400;
//    private static final int UP_ENCODER = 400;
    public  double MAX_LIFT_ENCODER = 10000;
    public  double MIN_LIFT_ENCODER = -10000;
    public static int TOP_ENCODER = 0;
    public static int BOTTOM_ENCODER = 0;
    public static int FLIP_UP_ENCODER = -303;
    public static int FLIP_DOWN_ENCODER = 400;



    private  double setPoint = 0;
    private  double flipSetPoint = 0;
    private double potentiometerTop=180;
    private double potentiometerBottom=0;


    private boolean topPressed = false, bottomPressed = false;
    private boolean flipPressed=false;
    private double liftEncoder = 0;
    private double flipEncoder = 0;

    public double getLiftEncoderValue() {
        return liftEncoder;
    }
    public double getFlipEncoderValue() {
        return flipEncoder;
    }

    public double getSetPoint(){return setPoint;}
    public double getFlipSetPoint(){return flipSetPoint;}


    public boolean getTopLimit() {
        return topPressed;
    }

    public boolean getBottomLimit() {
        return bottomPressed;
    }
    public boolean getFlipLimit() {
        return flipPressed;
    }


    private final StepTimer stepTimer = new StepTimer("grabber", Log.VERBOSE);

    public void act() {

        stepTimer.start();
        stepTimer.step("topLimit");

        topLimit.update();
        flipLimit.update();
        stepTimer.step("bottomLimit");

        bottomLimit.update();
        stepTimer.step("logic");

        flipEncoder=flip.getEncoderPosition();
        liftEncoder = lift.getEncoderPosition();


//        if(flipEncoder>=)

        topPressed = topLimit.justPressed();
        bottomPressed = bottomLimit.isPressed();
        flipPressed=flipLimit.isPressed();
        if(bottomPressed==true){
            lift.resetEncoder();
            MIN_LIFT_ENCODER =0;

        }
        else if(topPressed==true){
            MAX_LIFT_ENCODER =lift.getEncoderPosition();
        }
        if(setPoint> MAX_LIFT_ENCODER){
            setPoint= MAX_LIFT_ENCODER;
        }

        else if(setPoint< MIN_LIFT_ENCODER){
            setPoint= MIN_LIFT_ENCODER;
        }



        if(flipPressed==true){
            flip.resetEncoder();
            FLIP_DOWN_ENCODER =0;

        }

//
        if(flipSetPoint >FLIP_DOWN_ENCODER){
            flipSetPoint= FLIP_DOWN_ENCODER;
        }
        else if(flipSetPoint< FLIP_UP_ENCODER){
            flipSetPoint= FLIP_UP_ENCODER;

        }
        lift.setPosition((int) setPoint,1);

//        lift.setPower(liftPID.computeCorrection(setPoint,liftEncoder));
        flip.setPosition((int) flipSetPoint,.75);

//        else if(mode==Mode.DOWN){
//            lift.resetEncoder();
//            lift.setPower(0);
//        }

//        else if(mode==Mode.GO_MIDDLE){
//
//            setPoint=UPMID_ENCODER;
//            mode=Mode.MIDDLE;
////            if(lift.getEncoderPosition())
////            if(lift.getEncoderPosition()>UPMID_ENCODER){
////            lift.setSpeed(-LIFT_SPEED);}
////            else if(lift.getEncoderPosition()<UPMID_ENCODER){
////                lift.setSpeed(LIFT_SPEED);
////            }
////            else if (lift.getEncoderPosition()  UPMID_ENCODER) {
////                setPoint=lift.getEncoderPosition();
////                mode=Mode.MIDDLE;
////            }
//        }
//        else if(mode==Mode.GO_UP){
//            setPoint=UP_ENCODER;
//
//            if (topLimit.isPressed()) {
//                mode = Mode.UP;
//            }
//        }
//        else if(mode==Mode.GO_DOWN){
//            lift.setPower(-DOWN_LIFT_SPEED);
//            if (bottomLimit.isPressed()) {
//                lift.setPower(0);
//                mode = Mode.DOWN;
//            }
//        }
//        else if(mode==Mode.GO_LITTLE_UP){
//            lift.setPower(LIFT_SPEED);
//            setPoint=LITTLE_UP;
//            mode=Mode.LITTLE_UP;
//        }
//





//        if(servoAction!=lastServoAction) {
//            lastServoAction=servoAction;
//
//            if (servoAction == ServoAction.EJECT_All) {
//                topLeftRelease.goToPreset(RobotCfg2017.TopLeftReleaseServoPresets.EJECT);
//                topRightRelease.goToPreset(RobotCfg2017.TopRightReleaseServoPresets.EJECT);
//                bottomLeftRelease.goToPreset(RobotCfg2017.BottomLeftReleaseServoPresets.EJECT);
//                bottomRightRelease.goToPreset(RobotCfg2017.BottomRightReleaseServoPresets.EJECT);
//
//            } else if (servoAction == ServoAction.HALT_ALL) {
//                topLeftRelease.goToPreset(RobotCfg2017.TopLeftReleaseServoPresets.HALT);
//                topRightRelease.goToPreset(RobotCfg2017.TopRightReleaseServoPresets.HALT);
//                bottomLeftRelease.goToPreset(RobotCfg2017.BottomLeftReleaseServoPresets.HALT);
//                bottomRightRelease.goToPreset(RobotCfg2017.BottomRightReleaseServoPresets.HALT);
//
//            } else if (servoAction == ServoAction.GRAB_All) {
//                topLeftRelease.goToPreset(RobotCfg2017.TopLeftReleaseServoPresets.GRAB);
//                topRightRelease.goToPreset(RobotCfg2017.TopRightReleaseServoPresets.GRAB);
//                bottomLeftRelease.goToPreset(RobotCfg2017.BottomLeftReleaseServoPresets.GRAB);
//                bottomRightRelease.goToPreset(RobotCfg2017.BottomRightReleaseServoPresets.GRAB);
//
//            }
//            else if (servoAction == ServoAction.BOTTOM_ONLY_EJECT) {
//                topLeftRelease.goToPreset(RobotCfg2017.TopLeftReleaseServoPresets.HALT);
//                topRightRelease.goToPreset(RobotCfg2017.TopRightReleaseServoPresets.HALT);
//                bottomLeftRelease.goToPreset(RobotCfg2017.BottomLeftReleaseServoPresets.EJECT);
//                bottomRightRelease.goToPreset(RobotCfg2017.BottomRightReleaseServoPresets.EJECT);
//
//            }
//            else if (servoAction == ServoAction.TOP_ONLY_EJECT) {
//                topLeftRelease.goToPreset(RobotCfg2017.TopLeftReleaseServoPresets.EJECT);
//                topRightRelease.goToPreset(RobotCfg2017.TopRightReleaseServoPresets.EJECT);
//                bottomLeftRelease.goToPreset(RobotCfg2017.BottomLeftReleaseServoPresets.HALT);
//                bottomRightRelease.goToPreset(RobotCfg2017.BottomRightReleaseServoPresets.HALT);
//
//            }
//            else if (servoAction == ServoAction.BOTTOM_ONLY_GRAB) {
//                topLeftRelease.goToPreset(RobotCfg2017.TopLeftReleaseServoPresets.HALT);
//                topRightRelease.goToPreset(RobotCfg2017.TopRightReleaseServoPresets.HALT);
//                bottomLeftRelease.goToPreset(RobotCfg2017.BottomLeftReleaseServoPresets.GRAB);
//                bottomRightRelease.goToPreset(RobotCfg2017.BottomRightReleaseServoPresets.GRAB);
//
//            }
//            else if (servoAction == ServoAction.TOP_ONLY_GRAB) {
//                topLeftRelease.goToPreset(RobotCfg2017.TopLeftReleaseServoPresets.GRAB);
//                topRightRelease.goToPreset(RobotCfg2017.TopRightReleaseServoPresets.GRAB);
//                bottomLeftRelease.goToPreset(RobotCfg2017.BottomLeftReleaseServoPresets.HALT);
//                bottomRightRelease.goToPreset(RobotCfg2017.BottomRightReleaseServoPresets.HALT);
//
//            }
//            else if (servoAction == ServoAction.TOP_LEFT) {
//                topLeftRelease.goToPreset(RobotCfg2017.TopLeftReleaseServoPresets.GRAB);
//                topRightRelease.goToPreset(RobotCfg2017.TopRightReleaseServoPresets.HALT);
//                bottomLeftRelease.goToPreset(RobotCfg2017.BottomLeftReleaseServoPresets.HALT);
//                bottomRightRelease.goToPreset(RobotCfg2017.BottomRightReleaseServoPresets.HALT);
//
//            }
//            else if (servoAction == ServoAction.TOP_RIGHT) {
//                topLeftRelease.goToPreset(RobotCfg2017.TopLeftReleaseServoPresets.HALT);
//                topRightRelease.goToPreset(RobotCfg2017.TopRightReleaseServoPresets.GRAB);
//                bottomLeftRelease.goToPreset(RobotCfg2017.BottomLeftReleaseServoPresets.HALT);
//                bottomRightRelease.goToPreset(RobotCfg2017.BottomRightReleaseServoPresets.HALT);
//
//            }
//            else if (servoAction == ServoAction.BOTTOM_LEFT) {
//                topLeftRelease.goToPreset(RobotCfg2017.TopLeftReleaseServoPresets.HALT);
//                topRightRelease.goToPreset(RobotCfg2017.TopRightReleaseServoPresets.HALT);
//                bottomLeftRelease.goToPreset(RobotCfg2017.BottomLeftReleaseServoPresets.GRAB);
//                bottomRightRelease.goToPreset(RobotCfg2017.BottomRightReleaseServoPresets.HALT);
//
//            }
//            else if (servoAction == ServoAction. BOTTOM_RIGHT) {
//                topLeftRelease.goToPreset(RobotCfg2017.TopLeftReleaseServoPresets.HALT);
//                topRightRelease.goToPreset(RobotCfg2017.TopRightReleaseServoPresets.HALT);
//                bottomLeftRelease.goToPreset(RobotCfg2017.BottomLeftReleaseServoPresets.HALT);
//                bottomRightRelease.goToPreset(RobotCfg2017.BottomRightReleaseServoPresets.GRAB);
//
//            }
//
//        }


        stepTimer.step("lift");

        lift.update();
        flip.update();


    stepTimer.stop();



    }


    public void goTop() {
        setPoint= MAX_LIFT_ENCODER;
    }

    public void goBottom() {        setPoint= MIN_LIFT_ENCODER;
    }
    public void goLittleUp() {        setPoint=LITTLE_UP;
    }

    public void goMiddle(){setPoint=UPMID_ENCODER;}
    public void goPoint(double point){setPoint=point;}
//    public void goFlipMoveUp(){setPoint=point;}

    public void goFlipUp(){
        flipSetPoint= FLIP_UP_ENCODER;}
    public void goFlipDown(){flipSetPoint= FLIP_DOWN_ENCODER;
    }

    public void goFlipPoint(double flipPoint){flipSetPoint=flipPoint;}

//    public double getPotentiometer(){return potentiometer.getValue();}



//    public ServoAction getServo() {
//        return servoAction;
//    }



//    public void haltAllServo() {
//       servoAction=ServoAction.HALT_ALL;
//    }
//    public void ejectAllServo() {
//        servoAction = ServoAction.EJECT_All;
//    }
//    public void grabAllServo() {
//
//    }
    public void topGrabServo() {
        topLeftRelease.goToPreset(RobotCfg2017.TopLeftReleaseServoPresets.GRAB);
        topRightRelease.goToPreset(RobotCfg2017.TopRightReleaseServoPresets.GRAB);


    }
    public void bottomGrabServo() {
        bottomLeftRelease.goToPreset(RobotCfg2017.BottomLeftReleaseServoPresets.GRAB);
        bottomRightRelease.goToPreset(RobotCfg2017.BottomRightReleaseServoPresets.GRAB);

    }
    public void topEjectServo() {
        topLeftRelease.goToPreset(RobotCfg2017.TopLeftReleaseServoPresets.EJECT);
        topRightRelease.goToPreset(RobotCfg2017.TopRightReleaseServoPresets.EJECT);

    }
    public void bottomEjectServo() {
        bottomRightRelease.goToPreset(RobotCfg2017.BottomRightReleaseServoPresets.EJECT);
        bottomLeftRelease.goToPreset(RobotCfg2017.BottomLeftReleaseServoPresets.EJECT);
    }
    public void topHaltServo() {
        topLeftRelease.goToPreset(RobotCfg2017.TopLeftReleaseServoPresets.HALT);
        topRightRelease.goToPreset(RobotCfg2017.TopRightReleaseServoPresets.HALT);

    }
    public void bottomHaltServo() {
        bottomRightRelease.goToPreset(RobotCfg2017.BottomRightReleaseServoPresets.HALT);
        bottomLeftRelease.goToPreset(RobotCfg2017.BottomLeftReleaseServoPresets.HALT);
    }
    public void topRightServo() {
        topRightRelease.goToPreset(RobotCfg2017.TopRightReleaseServoPresets.GRAB);

    }
    public void topLeftServo() {
        topLeftRelease.goToPreset(RobotCfg2017.TopRightReleaseServoPresets.GRAB);


    }
    public void bottomRightServo() {
        bottomRightRelease.goToPreset(RobotCfg2017.BottomRightReleaseServoPresets.GRAB);

    }
    public void bottomLeftServo() {
        bottomLeftRelease.goToPreset(RobotCfg2017.BottomLeftReleaseServoPresets.GRAB);
    }
    public void topRightServoHalt() {
        topRightRelease.goToPreset(RobotCfg2017.TopRightReleaseServoPresets.HALT);

    }
    public void topLeftServoHalt() {
        topLeftRelease.goToPreset(RobotCfg2017.TopRightReleaseServoPresets.HALT);


    }
    public void bottomRightServoHalt() {
        bottomRightRelease.goToPreset(RobotCfg2017.BottomRightReleaseServoPresets.HALT);

    }
    public void bottomLeftServoHalt() {
        bottomLeftRelease.goToPreset(RobotCfg2017.BottomLeftReleaseServoPresets.HALT);
    }
    public void toggleFlip() {
        if (flipAction== FlipAction.DOWN) {
            flipAction= FlipAction.UP;
            goFlipUp();
        } else if (flipAction== FlipAction.UP) {
            flipAction= FlipAction.DOWN;
            goFlipDown();

        }


    }

//    public void toggleServo() {
//        if (servoAction == ServoAction.HALT ) {
//            grabServo();
//        } else  {
//            haltServo();
//        }
//    }









}
