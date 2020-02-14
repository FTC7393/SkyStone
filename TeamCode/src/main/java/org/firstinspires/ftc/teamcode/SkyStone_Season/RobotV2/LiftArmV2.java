package org.firstinspires.ftc.teamcode.SkyStone_Season.RobotV2;

import org.firstinspires.ftc.teamcode.SkyStone_Season.SkystoneRobotCfg;
import org.firstinspires.ftc.teamcode.SkyStone_Season.TeleOp.LiftArmStates;
import org.firstinspires.ftc.teamcode.SkyStone_Season.TeleOp.LinearSlide;

import ftc.electronvolts.statemachine.BasicAbstractState;
import ftc.electronvolts.statemachine.StateMachine;
import ftc.electronvolts.statemachine.StateMachineBuilder;
import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.util.BasicResultReceiver;
import ftc.electronvolts.util.InputExtractor;
import ftc.electronvolts.util.PIDController;
import ftc.electronvolts.util.ResultReceiver;
import ftc.evlib.hardware.motors.MotorEnc;
import ftc.evlib.hardware.sensors.DigitalSensor;
import ftc.evlib.hardware.servos.ServoControl;
import ftc.evlib.statemachine.EVStates;

public class LiftArmV2 {

    private final ServoControl wrist, gripper, fingerLeft, fingerRight;



    private final DigitalSensor lowerLimitVerticalRight;
    private final DigitalSensor lowerLimitVerticalLeft;
    private final DigitalSensor lowerLimitHorizontal;
    private boolean isArmExtended = false;
    private LinearSlide HorizontalSlide;
    private LinearSlide VerticalSlideRight;
    private LinearSlide VerticalSlideLeft;
//    private StateMachine stateMachine;
    private double LiftCommand;
    private double ExtensionCommand;
    private double WristCommand;
    private final int VerticalMaxExtension = 3500;
    private final int HorizontalMaxExtension = 3500;
    private final int LiftToleranceHorizontal = 5;
    private final int LiftToleranceVertical = 5;
    private final int LiftKeepOutUpperLimit = 50;
    private final int LiftKeepOutInnerLimit = 50;
    private final int LiftKeepOutOuterLimit = 50;
    private final int WristKeepOutWristLimit = 50;
    private final int WristKeepOutInnerLimit = 50;
    private final int WristKeepOutOuterLimit = 50;
    private final int numberOfLevels = 6; //random number, don't know actual value yet. TODO
    private final int placingLevelHeights[] = {100, 600, 1100, 1600, 2100, 7000}; //random number, don't know actual value yet. TODO
    private final int droppingLevelHeights[] = {0, 500, 1000, 1500, 2000, 5700}; //random number, don't know actual value yet. TODO


    private final double elbowSpeed = 0.6;
    private final double wristSpeed = 0.6;
    private final double fingerSpeed = 1.15;

    public LiftArmV2(ServoControl wrist, ServoControl gripper, ServoControl fingerRight, ServoControl fingerLeft, MotorEnc VerticalRightMotor,
                     MotorEnc VerticalLeftMotor, MotorEnc HorizontalMotor, DigitalSensor lowerLimitVerticalRight, DigitalSensor lowerLimitVerticalLeft, DigitalSensor lowerLimitHorizontal) {
        this.wrist = wrist;
        this.gripper = gripper;
        this.fingerLeft = fingerLeft;
        this.fingerRight = fingerRight;
        this.VerticalSlideLeft = new LinearSlide(VerticalLeftMotor, new PIDController(0.003, 0, 0, .5),
                VerticalMaxExtension, LiftToleranceVertical, lowerLimitVerticalRight);
        this.VerticalSlideRight = new LinearSlide(VerticalRightMotor, new PIDController(0.003, 0, 0, .5),
                VerticalMaxExtension, LiftToleranceVertical, lowerLimitVerticalLeft);
        this.HorizontalSlide = new LinearSlide(HorizontalMotor, new PIDController(0.003, 0, 0, .5),
                HorizontalMaxExtension, LiftToleranceHorizontal, lowerLimitHorizontal);
        this.lowerLimitVerticalRight = lowerLimitVerticalRight;
        this.lowerLimitVerticalLeft= lowerLimitVerticalLeft;
        this.lowerLimitHorizontal = lowerLimitHorizontal;
//        this.rrCommand = new BasicResultReceiver<>();
//        this.rrPlacingHeight = new InputExtractor<Integer>(){
//            @Override
//            public Integer getValue() {
//                return getPlacingHeight();
//            }
//        };
//        this.rrLowerLimitResetComplete = new InputExtractor<Boolean>() {
//            @Override
//            public Boolean getValue() {
//                return lift.getLowerLimitResetComplete();
//            }
//        };
//        this.rrMinExtensionPosition = new InputExtractor<Integer>() {
//            @Override
//            public Integer getValue() {
//                return lift.getMinExtensionValue();
//            }
//        };
//        this.rrDroppingHeight = new InputExtractor<Integer>() {
//            @Override
//            public Integer getValue() {
//                return getDroppingHeight();
//            }
//        };
//        this.rrAboveEmptySafeHeight = new BasicResultReceiver<Boolean>() {
//            @Override
//            public boolean isReady() {
//                return true;
//            }
//
//            @Override
//            public Boolean getValue() {
//                return lift.getExtensionEncoder() >= emptySafeHeight-liftTolerance;
//            }
//        };
//
//        this.stateMachine = buildStates();
    }
    public DigitalSensor getLowerLimitVerticalLeft() {
        return lowerLimitVerticalLeft;
    }
    public DigitalSensor getLowerLimitHorizontal() {
        return lowerLimitHorizontal;
    }
    public DigitalSensor getLowerLimitVerticalRight() {
        return lowerLimitVerticalRight;
    }
    public void controlExtension(double extensionDelta) {
        double newCommand = HorizontalSlide.getExtensionEncoder() + extensionDelta;
        if (VerticalSlideRight.getExtensionEncoder() < LiftKeepOutUpperLimit && VerticalSlideLeft.getExtensionEncoder() < LiftKeepOutUpperLimit) {
            if (ExtensionCommand <= LiftKeepOutInnerLimit && newCommand > LiftKeepOutInnerLimit) {
                newCommand = LiftKeepOutInnerLimit;
            } else if (ExtensionCommand >= LiftKeepOutOuterLimit && newCommand < LiftKeepOutOuterLimit) {
                newCommand = LiftKeepOutOuterLimit;
            }
        }
        if (wrist.getCurrentPosition() > WristKeepOutWristLimit) {
            if (ExtensionCommand > WristKeepOutOuterLimit && newCommand < WristKeepOutOuterLimit) {
                newCommand = WristKeepOutOuterLimit;
            }
        }
        ExtensionCommand = newCommand;
        HorizontalSlide.setExtension(ExtensionCommand);
    }

    public void controlLift(double liftDelta) {
        double newCommand = ((VerticalSlideLeft.getExtensionEncoder() + VerticalSlideRight.getExtensionEncoder()) / 2) + liftDelta;
        if (HorizontalSlide.getExtensionEncoder() >= LiftKeepOutInnerLimit && HorizontalSlide.getExtensionEncoder() <= LiftKeepOutOuterLimit) {
            if (LiftCommand >= LiftKeepOutUpperLimit && newCommand < LiftKeepOutUpperLimit) {
                newCommand = LiftKeepOutUpperLimit;
            }
        }
        LiftCommand = newCommand;
        VerticalSlideLeft.setExtension(LiftCommand);
        VerticalSlideRight.setExtension(LiftCommand);
    }

    public void fingersIngest() {
        fingerLeft.goToPreset(SkystoneRobotCfgV2.FingerLeftServoPresets.FORWARD);
        fingerRight.goToPreset(SkystoneRobotCfgV2.FingerRightServoPresets.FORWARD);
    }

    public void fingerEject() {
        fingerLeft.goToPreset(SkystoneRobotCfgV2.FingerLeftServoPresets.BACKWARD);
        fingerRight.goToPreset(SkystoneRobotCfgV2.FingerRightServoPresets.BACKWARD);
    }

    public void fingersRight() {
        fingerLeft.goToPreset(SkystoneRobotCfgV2.FingerLeftServoPresets.BACKWARD);
        fingerRight.goToPreset(SkystoneRobotCfgV2.FingerRightServoPresets.FORWARD);
    }

    public void fingersLeft() {
        fingerLeft.goToPreset(SkystoneRobotCfgV2.FingerLeftServoPresets.FORWARD);
        fingerRight.goToPreset(SkystoneRobotCfgV2.FingerRightServoPresets.BACKWARD);
    }

    public void fingersStop() {
        fingerLeft.goToPreset(SkystoneRobotCfgV2.FingerLeftServoPresets.STOP);
        fingerRight.goToPreset(SkystoneRobotCfgV2.FingerRightServoPresets.STOP);
    }

    public void wrist90() {
        if (HorizontalSlide.getExtensionEncoder() > WristKeepOutOuterLimit) {
            wrist.goToPreset(SkystoneRobotCfgV2.WristServoPresets.NINETY);
        }
    }

    public void gripperGrab() {
        gripper.goToPreset(SkystoneRobotCfgV2.GripperServoPresets.GRAB);
    }

    public void gripperRelease() {
        gripper.goToPreset(SkystoneRobotCfgV2.GripperServoPresets.RELEASE);
    }

    public void wrist0() {
        wrist.goToPreset(SkystoneRobotCfgV2.WristServoPresets.ZERO);
    }

    public double getLiftPosition() {
        return ((VerticalSlideLeft.getExtensionEncoder() + VerticalSlideRight.getExtensionEncoder()) / 2);
    }

    public void setLiftPosition(double liftPosition) {
        VerticalSlideLeft.setExtension(liftPosition);
        VerticalSlideRight.setExtension(liftPosition);
    }


    public boolean isDone() {
        return HorizontalSlide.isDone() && VerticalSlideRight.isDone() && VerticalSlideLeft.isDone() && wrist.isDone() && gripper.isDone();
    }

    public boolean verticalIsDone() {
       return VerticalSlideRight.isDone() && VerticalSlideLeft.isDone();
    }

    public boolean horizontalIsDone() {
        return HorizontalSlide.isDone();
    }

    public boolean handIsDone() {
       return wrist.isDone() && gripper.isDone();
    }

    public void pre_act() {
        HorizontalSlide.pre_act();
        VerticalSlideLeft.pre_act();
        VerticalSlideRight.pre_act();
    }

    public void act() {
//        stateMachine.act();
        VerticalSlideRight.act();
        VerticalSlideLeft.act();
        HorizontalSlide.act();
        wrist.act();
        gripper.act();
        fingerLeft.act();
        fingerRight.act();
    }

    public void stop(){
        VerticalSlideRight.stopExtension();
        VerticalSlideLeft.stopExtension();
        HorizontalSlide.stopExtension();
    }
}

    /////////////////////////
    ///stateMachineSection///
    /////////////////////////

//    private final ResultReceiver<COMMANDS> rrCommand;
//
//    public void sendCommand(COMMANDS newCommand){
//        rrCommand.setValue(newCommand);
//    }
//
//    private final InputExtractor<Integer> rrPlacingHeight;
//
//    private final InputExtractor<Integer> rrDroppingHeight;
//
//    private final InputExtractor<Boolean> rrLowerLimitResetComplete;
//
//    private final InputExtractor<Integer> rrMinExtensionPosition;
//
//    private final ResultReceiver<Boolean> rrAboveEmptySafeHeight;
//
//    private int placingLevel = 0;
//
//    public int incrementPlacingLevel() {
//        int n = (int)Math.floor(findFractionalLevel());
//        if (n < numberOfLevels-1){
//            placingLevel = n+1;
//        }
//        lift.setExtension(placingLevelHeights[placingLevel]);
//        return placingLevel;
//    }
//
//    public int decrementPlacingLevel() {
//        int n = (int)Math.ceil(findFractionalLevel());
//        if (n > 0){
//            placingLevel = n-1;
//        }
//        lift.setExtension(placingLevelHeights[placingLevel]);
//        return placingLevel;
//    }
//
//    private double findFractionalLevel() {
//        long r = Math.round(lift.getExtensionSetPoint());
//        int i = 0;
//        if (r <= placingLevelHeights[0]){
//            return 0;
//        }
//        for(i = 1; i < numberOfLevels; i++) {
//            if(r < placingLevelHeights[i]) {
//                return (double)(r-placingLevelHeights[i-1])/(placingLevelHeights[i]-placingLevelHeights[i-1])+i-1;
//            }
//            if(r == placingLevelHeights[i]) {
//                return i;
//            }
//        }
//        return numberOfLevels - 1;
//    }
//
//    public void setPlacingLevel(int placingLevel) {
//        if(placingLevel >= 0 && placingLevel < numberOfLevels){
//            this.placingLevel = placingLevel;
//        }
//    }
//
//    public StateName getCurrentStateName() {
//        return stateMachine.getCurrentStateName();
//    }
//
//    public int getPlacingLevel() {
//        return this.placingLevel;
//    }
//
//    public int getPlacingHeight() {
//        return placingLevelHeights[placingLevel];
//    }
//
//    public int getDroppingHeight() {
//        return droppingLevelHeights[placingLevel];
//    }
//
//    public StateMachine buildStates(){
//        StateMachineBuilder b = new StateMachineBuilder(S.INIT);
//
//
//
//        b.add(S.INIT, new BasicAbstractState() {
//            @Override
//            public void init() {
//                rrCommand.clear();
//            }
//
//            @Override
//            public boolean isDone() {
//                return rrCommand.isReady();
//            }
//
//            @Override
//            public StateName getNextStateName() {
//                if (rrCommand.getValue() != null){
//                    if (rrLowerLimitResetComplete.getValue() == true) {
//                        // The limit reset has been completed
//                        switch(rrCommand.getValue()){
//                            case STOW:
//                                return S.MOVE_A5_1;
//
//                        }
//                    }else{
//                        // lower  limit has not been reset, position is unknown
//                        switch(rrCommand.getValue()){
//                            case STOW:
//                                return S.MOVE_C1_1;
//
//                        }
//                    }
//
//                }
//                return null;
//            }
//        });
//
//
//
//
//        b.add(S.STOWED, new BasicAbstractState() {
//            @Override
//            public void init() {
//                rrCommand.clear();
//            }
//
//            @Override
//            public boolean isDone() {
//                return rrCommand.isReady();
//            }
//
//            @Override
//            public StateName getNextStateName() {
//                if (rrCommand.getValue() != null){
//                    switch(rrCommand.getValue()){
//                        case GRAB:
//                            return S.MOVE_A1_1;
//
//                        case PLACE:
//                            return S.MOVE_A2_1;
//
//                        case MANUAL:
//                            return S.MANUAL;
//                    }
//                }
//                return null;
//            }
//        });
//
//
//
//        b.add(S.GRABBED, new BasicAbstractState() {
//            @Override
//            public void init() {
//                rrCommand.clear();
//            }
//
//            @Override
//            public boolean isDone() {
//                return rrCommand.isReady();
//            }
//
//            @Override
//            public StateName getNextStateName() {
//                if (rrCommand.getValue() != null){
//                    switch(rrCommand.getValue()){
//                        case STOW:
//                            return S.MOVE_C1_1;
//
//                        case PLACE:
//                            return S.MOVE_A2_1;
//
//                        case MANUAL:
//                            return S.MANUAL;
//                    }
//                }
//                return null;
//            }
//        });
//
//        b.add(S.PLACED, new BasicAbstractState() {
//            @Override
//            public void init() {
//                rrCommand.clear();
//            }
//
//            @Override
//            public boolean isDone() {
//                return rrCommand.isReady();
//            }
//
//            @Override
//            public StateName getNextStateName() {
//                if (rrCommand.getValue() != null){
//                    switch(rrCommand.getValue()){
//                        case STOW:
//                            return S.MOVE_A5_1;
//
//                        case GRAB:
//                            return S.MOVE_B2_1;
//
//                        case DROP:
//                            return S.MOVE_A3_1;
//
//                        case PLACE:
//                            return S.MOVE_A2_1;
//
//                        case MANUAL:
//                            return S.MANUAL;
//                    }
//                }
//                return null;
//            }
//        });
//
//
//        b.add(S.DROPPED, new BasicAbstractState() {
//            @Override
//            public void init() {
//                //rrCommand.clear();
//            }
//
//            @Override
//            public boolean isDone() {
//                return rrCommand.isReady();
//            }
//
//            @Override
//            public StateName getNextStateName() {
//                if (rrCommand.getValue() != null){
//                    switch(rrCommand.getValue()){
//                        case PLACE:
//                            return S.MOVE_A4_1;
//
//                        case UNDROP:
//                            return S.MOVE_A4_1;
//
//                        case MANUAL:
//                            return S.MANUAL;
//
//                    }
//                }
//                return null;
//            }
//        });
//        b.add(S.MANUAL, new BasicAbstractState() {
//            @Override
//            public void init() {
//                //rrCommand.clear();
//            }
//
//            @Override
//            public boolean isDone() {
//                return rrCommand.isReady();
//            }
//
//            @Override
//            public StateName getNextStateName() {
//                if (rrCommand.getValue() != null){
//                    switch(rrCommand.getValue()){
//                        case STOW:
//                            return S.MOVE_A5_1;
//                    }
//                }
//                return null;
//            }
//        });
//
//
//
//        /*
//        Stowed ⟶ Grabbing A1:
//        - Move linear slide up to an empty safe height, wait for move to complete
//        - Elbow and wrist servos in grabbing position and finger servo is in open position
//        - Lover linear slide to a grabbing height, wait for move to complete
//        - Put finger servos in a closed position
//         */
//
//        b.add(S.MOVE_A1_1, EVStates.servoTurn(S.MOVE_A1_1A,
//                fingers, SkystoneRobotCfg.FingersServoPresets.RELEASE, fingerSpeed,false));
//        b.add(S.MOVE_A1_1A, LiftArmStates.liftMove(S.MOVE_A1_1B, this, stowedToGrabHeight, false));
//        b.add(S.MOVE_A1_1B, LiftArmStates.waitForLiftArm(S.MOVE_A1_2, this));
//        b.add(S.MOVE_A1_2, EVStates.servoTurn(S.MOVE_A1_3,
//                elbow, SkystoneRobotCfg.ElbowServoPresets.GRABBING, elbowSpeed, false));
//        b.add(S.MOVE_A1_3, EVStates.servoTurn(S.MOVE_A1_4,
//                wrist, SkystoneRobotCfg.WristServoPresets.GRABBING, wristSpeed, false));
//        b.add(S.MOVE_A1_4, LiftArmStates.waitForArm(S.MOVE_A1_5, this));
//        b.add(S.MOVE_A1_5, LiftArmStates.liftMove(S.MOVE_A1_6, this, grabbingHeight, true));
//        b.add(S.MOVE_A1_6, EVStates.servoTurn(S.GRABBED,
//                fingers, SkystoneRobotCfg.FingersServoPresets.GRAB, fingerSpeed,true));
//
//        /*
//        Grabbing ⟶ Stowed B1:
//        Put finger servos to open position
//        Move linear slide to empty safe height, wait
//        Elbow and wrist servos to stowed position and finger servos to closed position, wait
//        Move linear slide to stowed position
//         */
//
//        b.add(S.MOVE_B1_1, EVStates.servoTurn(S.MOVE_B1_2,
//                fingers, SkystoneRobotCfg.FingersServoPresets.RELEASE, fingerSpeed,true));
//        b.add(S.MOVE_B1_2, LiftArmStates.liftMove(S.MOVE_B1_3, this, stowedToGrabHeight, true));
//        b.add(S.MOVE_B1_3, EVStates.servoTurn(S.MOVE_B1_4,
//                elbow, SkystoneRobotCfg.ElbowServoPresets.STOWED, elbowSpeed,false));
//        b.add(S.MOVE_B1_4, EVStates.servoTurn(S.MOVE_B1_5,
//                wrist, SkystoneRobotCfg.WristServoPresets.STOWED, wristSpeed,false));
//        b.add(S.MOVE_B1_5, EVStates.servoTurn(S.MOVE_B1_6,
//                fingers, SkystoneRobotCfg.FingersServoPresets.GRAB, fingerSpeed,false));
//        b.add(S.MOVE_B1_6, LiftArmStates.waitForArm(S.MOVE_B1_7, this));
//        b.add(S.MOVE_B1_7, LiftArmStates.liftMove(S.STOWED, this, stowedHeight, true));
//
//
//        /*
//        Init(encoder: unknown) ⟶ Stowed C1:
//        Release the grabber
//        Move the slide to empty safe height; knowing it will go somewhere higher than that; if too high it will hit to limits switch :)
//        After it is safe to move servos to stowed position
//        Drop the slide/lift to a value equal to min
//        Move the slide to stowed height
//         */
//        b.add(S.MOVE_C1_1, EVStates.servoTurn(S.MOVE_C1_2,
//                fingers, SkystoneRobotCfg.FingersServoPresets.RELEASE,fingerSpeed,true));
//        b.add(S.MOVE_C1_2, LiftArmStates.liftMove(S.MOVE_C1_3, this, emptySafeHeight, true));
//        b.add(S.MOVE_C1_3, EVStates.servoTurn(S.MOVE_C1_4,
//                elbow, SkystoneRobotCfg.ElbowServoPresets.STOWED,elbowSpeed, false));
//        b.add(S.MOVE_C1_4, EVStates.servoTurn(S.MOVE_C1_5,
//                wrist, SkystoneRobotCfg.WristServoPresets.STOWED,wristSpeed,false));
//        b.add(S.MOVE_C1_5, EVStates.servoTurn(S.MOVE_C1_6,
//                fingers, SkystoneRobotCfg.FingersServoPresets.GRAB,fingerSpeed,false));
//        b.add(S.MOVE_C1_6, LiftArmStates.waitForArm(S.MOVE_C1_7, this));
//        b.add(S.MOVE_C1_7, LiftArmStates.liftMove(S.MOVE_C1_8, this, rrMinExtensionPosition, true));
//        b.add(S.MOVE_C1_8, LiftArmStates.liftMove(S.STOWED, this, stowedHeight, true));
//
////        Grabbing ⟶ Placing A2:
////        Move linear slide up to a loaded safe height, wait for move to complete
////        If current placing height is >= loaded safe height
////        Move linear slide to placing height, don’t wait
////        Move elbow and wrist servos to placing position
////        Wait for servos and slide to finish moving
////        If current placing height is < loaded safe height
////        Move elbow and wrist servos to placing position, wait
////        Move linear slide to placing height, wait
////        This works for both Grabbing ⟶ Placing A2: and Stowed ⟶ Placing B3:
//        b.add(S.MOVE_A2_1, LiftArmStates.liftMove(S.MOVE_A2_2, this, loadedSafeHeight, true));
//        b.add(S.MOVE_A2_2, EVStates.servoTurn(S.MOVE_A2_3,
//                elbow, SkystoneRobotCfg.ElbowServoPresets.PLACING,elbowSpeed,false));
//        b.add(S.MOVE_A2_3, EVStates.servoTurn(S.MOVE_A2_3A,
//                wrist, SkystoneRobotCfg.WristServoPresets.PLACING,wristSpeed,false));
//        b.addWait(S.MOVE_A2_3A, S.MOVE_A2_4, 500);
//        b.add(S.MOVE_A2_4, LiftArmStates.waitForArm(S.MOVE_A2_5, this));
//        b.add(S.MOVE_A2_5, LiftArmStates.liftMove(S.PLACED, this, rrPlacingHeight, true));
//
//
//
//
////    Placing ⟶ Dropping A3:
////    Move linear slide (placement height - droppingΔ)
////    Finger servos to open position
//        b.add(S.MOVE_A3_1, LiftArmStates.liftMove(S.MOVE_A3_2, this, rrDroppingHeight, true));
//        b.add(S.MOVE_A3_2, EVStates.servoTurn(S.DROPPED,
//                fingers, SkystoneRobotCfg.FingersServoPresets.RELEASE,fingerSpeed,true));
//
//
//
//
//
////    Dropping ⟶ Placing A4:
////    Move back up to placement height, wait
////    Put finger servos in closed position
//        b.add(S.MOVE_A4_1, LiftArmStates.liftMove(S.MOVE_A4_2, this, rrPlacingHeight , true));
//        b.add(S.MOVE_A4_2, EVStates.servoTurn(S.PLACED,
//                fingers, SkystoneRobotCfg.FingersServoPresets.GRAB,fingerSpeed,true));
//
//
//
//
//
////        Placing ⟶ Stowed A5:
////        If linear slide height >= empty safe height
////        Move linear slide to empty safe height, don’t wait
////        Move elbow and wrist servos to stowed position
////        If linear slide height < empty safe height
////        Move linear slide to empty safe height, wait
////        Move elbow and wrist servos to stowed position
////        Wait for slide and servos to finish moving
////        Move linear slide to stowed position
//        b.add(S.MOVE_A5_1, LiftArmStates.liftMove(S.MOVE_A5_1A, this, emptySafeHeight , false));
//        b.addBranch(S.MOVE_A5_1A, S.MOVE_A5_2, S.MOVE_A5_1A, S.MOVE_A5_1A, rrAboveEmptySafeHeight);
//        b.add(S.MOVE_A5_2, EVStates.servoTurn(S.MOVE_A5_3,
//                elbow, SkystoneRobotCfg.ElbowServoPresets.STOWED,elbowSpeed,false));
//        b.add(S.MOVE_A5_3, EVStates.servoTurn(S.MOVE_A5_4,
//                wrist, SkystoneRobotCfg.WristServoPresets.STOWED,wristSpeed,false));
////          b.addWait(S.MOVE_A5_4A, S.MOVE_A5_4, 500);
//        b.add(S.MOVE_A5_4, LiftArmStates.waitForArm(S.MOVE_A5_5, this));
//        b.add(S.MOVE_A5_5, LiftArmStates.liftMove(S.STOWED, this, stowedHeight , true));
//
//
////        Placing ⟶ Grabbing B2:
////        If linear slide height >= empty safe height
////        Move linear slide to empty safe height, don’t wait
////        If linear slide height < empty safe height
////        Move linear slide to empty safe height, wait
////        Move elbow and wrist servos to grabbing position, put finger servos to open position
////        Wait for slide and servos to finish moving
////        Move linear slide to grabbing position
////        Put finger servos to closed position
//        b.add(S.MOVE_B2_1, LiftArmStates.liftMove(S.MOVE_B2_2, this, emptySafeHeight , true));
//        b.add(S.MOVE_B2_2, EVStates.servoTurn(S.MOVE_B2_3,
//                elbow, SkystoneRobotCfg.ElbowServoPresets.GRABBING,elbowSpeed,false));
//        b.add(S.MOVE_B2_3, EVStates.servoTurn(S.MOVE_B2_4,
//                wrist, SkystoneRobotCfg.WristServoPresets.GRABBING,wristSpeed,false));
//        b.add(S.MOVE_B2_4, EVStates.servoTurn(S.MOVE_B2_4A,
//                fingers, SkystoneRobotCfg.FingersServoPresets.RELEASE,fingerSpeed,false));
//        b.addWait(S.MOVE_B2_4A, S.MOVE_B2_5, 500);
//        b.add(S.MOVE_B2_5, LiftArmStates.waitForArm(S.MOVE_B2_6, this));
//        b.add(S.MOVE_B2_6, LiftArmStates.liftMove(S.MOVE_B2_7, this, grabbingHeight , true));
//        b.add(S.MOVE_B2_7, EVStates.servoTurn(S.GRABBED,
//                fingers, SkystoneRobotCfg.FingersServoPresets.GRAB,fingerSpeed,true));
//
//        return b.build();
//    }
//
//
//    /*
//      A1: Stowed ⟶ Grabbing A1:
//      A2: Grabbing ⟶ Placing A2:
//      A3: Placing ⟶ Dropping A3:
//      A4: Dropping ⟶ Placing A4:
//      A5: Placing ⟶ Stowed A5:
//      B1: Grabbing ⟶ Stowed B1:
//      B2: Placing ⟶ Grabbing B2:
//      B3: Stowed ⟶ Placing B3:
//     */
//    public enum S implements StateName {
//        INIT,
//        STOWED,
//        GRABBED,
//        PLACED,
//        DROPPED,
//        MOVE_A1_1,
//        MOVE_A1_1A,
//        MOVE_A1_1B,
//        MOVE_A1_2,
//        MOVE_A1_3,
//        MOVE_A1_4,
//        MOVE_A1_5,
//        MOVE_A1_6,
//        MOVE_B1_1,
//        MOVE_B1_2,
//        MOVE_B1_3,
//        MOVE_B1_4,
//        MOVE_B1_5,
//        MOVE_B1_6,
//        MOVE_B1_7,
//        MOVE_C1_1,
//        MOVE_C1_2,
//        MOVE_C1_3,
//        MOVE_C1_4,
//        MOVE_C1_5,
//        MOVE_C1_6,
//        MOVE_C1_7,
//        MOVE_C1_8,
//        MOVE_A2_1,
//        MOVE_A2_2,
//        MOVE_A2_3,
//        MOVE_A2_3A,
//        MOVE_A2_4,
//        MOVE_A2_5,
//        MOVE_A3_1,
//        MOVE_A3_2,
//        MOVE_A4_1,
//        MOVE_A4_2,
//        MOVE_A4_3,
//        MOVE_A5_1,
//        MOVE_A5_2,
//        MOVE_A5_3,
//        MOVE_A5_4,
//        MOVE_A5_4A,
//        MOVE_A5_5,
//        MOVE_B2_1,
//        MOVE_B2_2,
//        MOVE_B2_3,
//        MOVE_B2_4,
//        MOVE_B2_4A,
//        MOVE_B2_5,
//        MOVE_B2_6,
//        MOVE_A5_1A, MANUAL, MOVE_B2_7
//    }
//
//    public enum COMMANDS {
//        STOW,
//        GRAB,
//        PLACE,
//        DROP,
//        UNDROP,
//        STOP,
//        MANUAL,
//        INITIALIZE,
//        DROP_LEFT,
//        PLACE_LEFT,
//        FORCE_STOW,
//        FORCE_GRAB,
//        FORCE_PLACE,
//        FORCE_DROP,
//        FORCE_PLACE_LEFT,
//        FORCE_DROP_LEFT
//    }
//    public void stop() {
//        lift.stopExtension();
//    }
//
//}