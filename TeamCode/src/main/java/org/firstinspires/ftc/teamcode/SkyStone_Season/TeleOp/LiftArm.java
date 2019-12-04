package org.firstinspires.ftc.teamcode.SkyStone_Season.TeleOp;

import org.firstinspires.ftc.teamcode.SkyStone_Season.SkystoneRobotCfg;

import ftc.electronvolts.statemachine.BasicAbstractState;
import ftc.electronvolts.statemachine.StateMachine;
import ftc.electronvolts.statemachine.StateMachineBuilder;
import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.util.InputExtractor;
import ftc.electronvolts.util.PIDController;
import ftc.evlib.hardware.config.RobotCfg;
import ftc.evlib.hardware.motors.MotorEnc;
import ftc.evlib.hardware.sensors.DigitalSensor;
import ftc.evlib.hardware.servos.ServoControl;
import ftc.electronvolts.util.BasicResultReceiver;
import ftc.electronvolts.util.ResultReceiver;
import ftc.evlib.statemachine.EVStates;

public class LiftArm {

    private final ServoControl elbow, wrist, fingers;
    private boolean isArmExtended = false;
    private LinearSlide lift;
    private StateMachine stateMachine;
    private final int maxExtensionPosition = 3522; //random number, don't know actual value yet. TODO
    private final int liftTolerance = 5;
    private final int safeArmExtensionPosition = maxExtensionPosition; // lift must be grater than a magic number to extend/retract arm.
    private final int emptySafeHeight = 2000; //random number, don't know actual value yet. TODO
    private final int grabbingHeight = 7000; //random number, don't know actual value yet. TODO
    private final int stowedHeight = 0; //random number, don't know actual value yet. TODO
    private final int loadedSafeHeight = 5700; //random number, don't know actual value yet. TODO
    private final int numberOfLevels = 5; //random number, don't know actual value yet. TODO
    private final int placingLevelHeights[] = {1000, 2000, 3000, 4000, 5000}; //random number, don't know actual value yet. TODO

    public LiftArm(ServoControl elbow, ServoControl wrist, ServoControl fingers, MotorEnc extension,
                   DigitalSensor lowerLimit, DigitalSensor upperLimit ) {
        this.elbow = elbow;
        this.wrist = wrist;
        this.fingers = fingers;
        this.lift = new LinearSlide(extension, new PIDController(0.0025, 0, 0, 1),
              maxExtensionPosition, liftTolerance, lowerLimit, upperLimit);
        this.rrCommand = new BasicResultReceiver<>();
        this.rrPlacingLevel = new InputExtractor<Integer>(){
            @Override
            public Integer getValue() {
                return getPlacingLevel();
            }
        };
        this.stateMachine = buildStates();
    }

    public void armPlacing() {
        if (lift.getExtensionEncoder() >=  safeArmExtensionPosition || isArmExtended) {
            elbow.goToPreset(SkystoneRobotCfg.ElbowServoPresets.PLACING);
            wrist.goToPreset(SkystoneRobotCfg.WristServoPresets.PLACING);
            isArmExtended = true;

        }
    }

    public void armPlacingLeft() {
        if (lift.getExtensionEncoder() >=  safeArmExtensionPosition || isArmExtended) {
            elbow.goToPreset(SkystoneRobotCfg.ElbowServoPresets.PLACING);
            wrist.goToPreset(SkystoneRobotCfg.WristServoPresets.PLACING_LEFT);
            isArmExtended = true;

        }
    }

    public void armGrabbing() {
        if (lift.getExtensionEncoder() >=  safeArmExtensionPosition) {
            elbow.goToPreset(SkystoneRobotCfg.ElbowServoPresets.GRABBING);
            wrist.goToPreset(SkystoneRobotCfg.WristServoPresets.GRABBING);
            isArmExtended = false;

        }
    }

    public void armStowed() {
        if (lift.getExtensionEncoder() >=  safeArmExtensionPosition) {
            elbow.goToPreset(SkystoneRobotCfg.ElbowServoPresets.STOWED);
            wrist.goToPreset(SkystoneRobotCfg.WristServoPresets.STOWED);
            isArmExtended = false;

        }
    }

    public void grab() {
        fingers.goToPreset(SkystoneRobotCfg.FingersServoPresets.GRAB);
    }

    public void release() { fingers.goToPreset(SkystoneRobotCfg.FingersServoPresets.RELEASE); }

    public LinearSlide getLift() {
        return lift;
    }

    public boolean isDone() {
        return lift.isDone() && elbow.isDone() && wrist.isDone() && fingers.isDone();
    }

    public void act() {
        stateMachine.act();
        elbow.act();
        wrist.act();
        fingers.act();
        lift.act();
    }

    /////////////////////////
    ///stateMachineSection///
    /////////////////////////

    private final ResultReceiver<COMMANDS> rrCommand;

    public void sendCommand(COMMANDS newCommand){
        rrCommand.setValue(newCommand);
    }

    private final InputExtractor<Integer> rrPlacingLevel;

    private int placingLevel = 0;

    public void setPlacingLevel(int placingLevel) {
        if(placingLevel >= 0 && placingLevel < numberOfLevels){
            this.placingLevel = placingLevel;
        }
    }

    public StateName getCurrentStateName() {
        return stateMachine.getCurrentStateName();
    }

    public int getPlacingLevel() {
        return this.placingLevel;
    }

    public StateMachine buildStates(){
        StateMachineBuilder b = new StateMachineBuilder(S.STOWED);
        b.add(S.STOWED, new BasicAbstractState() {
            @Override
            public void init() {
                rrCommand.clear();
            }

            @Override
            public boolean isDone() {
                return rrCommand.isReady();
            }

            @Override
            public StateName getNextStateName() {
                if (rrCommand.getValue() != null){
                    switch(rrCommand.getValue()){
                        case GRAB:
                            return S.MOVE_A1_1;

                        case FORCE_PLACE:

                        case FORCE_PLACE_LEFT:
                    }
                }
                return null;
            }
        });



        b.add(S.GRABBED, new BasicAbstractState() {
            @Override
            public void init() {
                rrCommand.clear();
            }

            @Override
            public boolean isDone() {
                return rrCommand.isReady();
            }

            @Override
            public StateName getNextStateName() {
                if (rrCommand.getValue() != null){
                    switch(rrCommand.getValue()){
                        case STOW:
                            return S.MOVE_B1_1;

                        case FORCE_PLACE:

                        case FORCE_PLACE_LEFT:
                    }
                }
                return null;
            }
        });

        /*
        Stowed ⟶ Grabbing A1:
        - Move linear slide up to an empty safe height, wait for move to complete
        - Elbow and wrist servos in grabbing position and finger servo is in open position
        - Lover linear slide to a grabbing height, wait for move to complete
        - Put finger servos in a closed position
         */

        b.add(S.MOVE_A1_1, LiftArmStates.liftMove(S.MOVE_A1_2, this, emptySafeHeight, true));
        b.add(S.MOVE_A1_2, EVStates.servoTurn(S.MOVE_A1_3,
                elbow, SkystoneRobotCfg.ElbowServoPresets.GRABBING,false));
        b.add(S.MOVE_A1_3, EVStates.servoTurn(S.MOVE_A1_4,
                wrist, SkystoneRobotCfg.WristServoPresets.GRABBING,false));
        b.add(S.MOVE_A1_4, LiftArmStates.waitForLiftArm(S.MOVE_A1_5, this));
        b.add(S.MOVE_A1_5, LiftArmStates.liftMove(S.MOVE_A1_6, this, grabbingHeight, true));
        b.add(S.MOVE_A1_6, EVStates.servoTurn(S.GRABBED,
                fingers, SkystoneRobotCfg.FingersServoPresets.GRAB,true));

        /*
        Grabbing ⟶ Stowed B1:
        Put finger servos to open position
        Move linear slide to empty safe height, wait
        Elbow and wrist servos to stowed position and finger servos to closed position, wait
        Move linear slide to stowed position
         */

        b.add(S.MOVE_B1_1, EVStates.servoTurn(S.MOVE_B1_2,
                fingers, SkystoneRobotCfg.FingersServoPresets.RELEASE,true));
        b.add(S.MOVE_B1_2, LiftArmStates.liftMove(S.MOVE_B1_3, this, emptySafeHeight, true));
        b.add(S.MOVE_B1_3, EVStates.servoTurn(S.MOVE_B1_4,
                elbow, SkystoneRobotCfg.ElbowServoPresets.STOWED,false));
        b.add(S.MOVE_B1_4, EVStates.servoTurn(S.MOVE_B1_5,
                wrist, SkystoneRobotCfg.WristServoPresets.STOWED,false));
        b.add(S.MOVE_B1_5, EVStates.servoTurn(S.MOVE_B1_6,
                fingers, SkystoneRobotCfg.FingersServoPresets.GRAB,false));
        b.add(S.MOVE_B1_6, LiftArmStates.waitForLiftArm(S.MOVE_B1_7, this));
        b.add(S.MOVE_B1_7, LiftArmStates.liftMove(S.STOWED, this, stowedHeight, true));

        return b.build();
    }

    /*
      A1: Stowed ⟶ Grabbing A1:
      A2: Grabbing ⟶ Placing A2:
      A3: Placing ⟶ Dropping A3:
      A4: Dropping ⟶ Placing A4:
      A5: Placing ⟶ Stowed A5:
      B1: Grabbing ⟶ Stowed B1:
      B2: Placing ⟶ Grabbing B2:
      B3: Stowed ⟶ Placing B3:
     */
    public enum S implements StateName {
        STOWED,
        GRABBED,
        PLACING,
        DROPPING,
        MOVE_A1_1,
        MOVE_A1_2,
        MOVE_A1_3,
        MOVE_A1_4,
        MOVE_A1_5,
        MOVE_A1_6,
        MOVE_B1_1,
        MOVE_B1_2,
        MOVE_B1_3,
        MOVE_B1_4,
        MOVE_B1_5,
        MOVE_B1_6,
        MOVE_B1_7
    }

    public enum COMMANDS {
        STOW,
        GRAB,
        PLACE,
        DROP,
        STOP,
        MANUAL,
        INITIALIZE,
        DROP_LEFT,
        PLACE_LEFT,
        FORCE_STOW,
        FORCE_GRAB,
        FORCE_PLACE,
        FORCE_DROP,
        FORCE_PLACE_LEFT,
        FORCE_DROP_LEFT,
    }
}