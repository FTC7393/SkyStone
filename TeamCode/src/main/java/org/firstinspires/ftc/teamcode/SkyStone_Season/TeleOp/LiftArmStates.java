package org.firstinspires.ftc.teamcode.SkyStone_Season.TeleOp;

import ftc.electronvolts.statemachine.BasicAbstractState;
import ftc.electronvolts.statemachine.State;
import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.statemachine.States;
import ftc.electronvolts.util.InputExtractor;

public class LiftArmStates extends States {

    public static State waitForLiftArm(final StateName nextStateName, final LiftArm liftArm){
        return new BasicAbstractState() {
            @Override
            public void init() {

            }

            @Override
            public boolean isDone() {
                return liftArm.isDone();
            }

            @Override
            public StateName getNextStateName() {
                return nextStateName;
            }
        };
    }

    public static State liftMove(final StateName nextStateName, final LiftArm liftArm, final double liftPosition, final boolean waitForDone){
        return new BasicAbstractState() {
            @Override
            public void init() {
                liftArm.getLift().setExtension(liftPosition);
            }

            @Override
            public boolean isDone() {
                if(waitForDone){
                    return liftArm.getLift().isDone();
                }else{
                    return true;
                }
            }

            @Override
            public StateName getNextStateName() {
                return nextStateName;
            }
        };
    }


    public static State liftMove(final StateName nextStateName, final LiftArm liftArm, final InputExtractor <Integer> liftPosition, final boolean waitForDone){
        return new BasicAbstractState() {
            @Override
            public void init() {
                liftArm.getLift().setExtension(liftPosition.getValue());
            }

            @Override
            public boolean isDone() {
                if(waitForDone){
                    return liftArm.getLift().isDone();
                }else{
                    return true;
                }
            }

            @Override
            public StateName getNextStateName() {
                return nextStateName;
            }
        };
    }
}


