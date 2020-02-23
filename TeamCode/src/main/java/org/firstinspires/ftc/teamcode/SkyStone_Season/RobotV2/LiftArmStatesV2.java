package org.firstinspires.ftc.teamcode.SkyStone_Season.RobotV2;

import org.firstinspires.ftc.teamcode.SkyStone_Season.TeleOp.LiftArm;

import ftc.electronvolts.statemachine.BasicAbstractState;
import ftc.electronvolts.statemachine.State;
import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.statemachine.States;
import ftc.electronvolts.util.InputExtractor;

public class LiftArmStatesV2 extends States {

    public static State waitForVertical (final StateName nextStateName, final LiftArmV2 liftArm){
        return new BasicAbstractState() {
            @Override
            public void init() {

            }

            @Override
            public boolean isDone() {
               return liftArm.verticalIsDone();
            }

            @Override
            public StateName getNextStateName() {
                return nextStateName;
            }
        };
    }

    public static State waitForHorizontal(final StateName nextStateName, final LiftArmV2 liftArm){
        return new BasicAbstractState() {
            @Override
            public void init() {

            }

            @Override
            public boolean isDone() {
                return liftArm.horizontalIsDone();
            }

            @Override
            public StateName getNextStateName() {
                return nextStateName;
            }
        };
    }

    public static State waitForHand(final StateName nextStateName, final LiftArmV2 liftArm){
        return new BasicAbstractState() {
            @Override
            public void init() {

            }

            @Override
            public boolean isDone() {
                return liftArm.handIsDone();
            }

            @Override
            public StateName getNextStateName() {
                return nextStateName;
            }
        };
    }

    public static State liftMove(final StateName nextStateName, final LiftArmV2 liftArm, final double liftPosition, final boolean waitForDone){
        return new BasicAbstractState() {
            @Override
            public void init() {
                liftArm.setLift((int)liftPosition);
            }

            @Override
            public boolean isDone() {
                if(waitForDone){
               return liftArm.verticalIsDone();
                }else{
                    return true;
                }
            }

            @Override
            public StateName getNextStateName() {
                return nextStateName;
            }
        };
    }}




