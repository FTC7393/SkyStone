package org.firstinspires.ftc.teamcode.Sri_TestBot_Skystone;

import com.google.common.collect.ImmutableList;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Abhi_Robot.TestBotCfg;

import ftc.electronvolts.statemachine.State;
import ftc.electronvolts.statemachine.StateMachine;
import ftc.electronvolts.statemachine.StateMachineBuilder;
import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.util.InputExtractor;
import ftc.electronvolts.util.files.Logger;
import ftc.evlib.hardware.motors.TwoMotors;
import ftc.evlib.opmodes.AbstractAutoOp;

@Autonomous(name = "TankDriveAuto")

public class TestBotAuto extends AbstractAutoOp<TestBotCfg> {


    @Override
    protected TestBotCfg createRobotCfg() {
        return new TestBotCfg(hardwareMap);
    }

    @Override
    protected Logger createLogger() {


        return new Logger("", "auto.csv",
                new ImmutableList.Builder<Logger.Column>()
                .add(new Logger.Column("State", new InputExtractor<StateName>() {

                    @Override
                    public StateName getValue() {
                        return stateMachine.getCurrentStateName();
                    }
                }))
                   .build()
        );
    }


    @Override
    protected void setup_act() {
    }

    @Override
    protected void go() {

    }


    @Override
    protected void act() {
        telemetry.addData("state", stateMachine.getCurrentStateName());

    }


    @Override
    protected void end() {
    }

    @Override
    public StateMachine buildStates() {


        StateMachineBuilder b = new StateMachineBuilder(S.DRIVE);
        b.add(S.DRIVE, new TwoMotorsRunForTime(robotCfg.getTwoMotors(), 0.8, S.STOP, 3000));
        b.addStop(S.STOP);


        return b.build();


    }


    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


        private enum S implements StateName {

            WAIT,
            DRIVE,
            OFF,
            STOP

        }

}

class waitState implements State {

    boolean isFirst = true;
    long timeAtStart = 0L;

    public waitState(StateName nextState, long waitTimeMillis) {
        this.nextState = nextState;
        this.waitTimeMillis = waitTimeMillis;
    }

    private final StateName nextState;
    private final long waitTimeMillis;

    @Override
    public StateName act() {

        if (isFirst) {
            timeAtStart = System.currentTimeMillis();
            isFirst = false;
            return null;
        }
        if(System.currentTimeMillis() - timeAtStart > waitTimeMillis) {
            return nextState;
        } else {
            return null;
        }
    }
}

class TwoMotorsRunForTime implements State {

    private final TwoMotors twoMotors;
    private final double power;
    private final StateName nextState;
    private final long runTimeMillis;
    private boolean isFirst = true;
    private long timeAtStart;


    public TwoMotorsRunForTime(TwoMotors twoMotors, double power, StateName nextState, long runTimeMillis) {
        this.twoMotors = twoMotors;
        this.power = power;
        this.nextState = nextState;
        this.runTimeMillis = runTimeMillis;
    }

    @Override
    public StateName act() {
        if (isFirst) {
            timeAtStart = System.currentTimeMillis();
            isFirst = false;
            twoMotors.runMotors(power, power);
            return null;
        }
        if(System.currentTimeMillis() - timeAtStart > runTimeMillis) {
            twoMotors.stop();
            return nextState;
        } else {
            return null;
        }
    }
}

class StopMotors implements State {

    private final TwoMotors twoMotors;
    private final StateName nextState;

    public StopMotors(TwoMotors twoMotors, StateName nextState) {
        this.twoMotors = twoMotors;
        this.nextState = nextState;
    }

    @Override
    public StateName act() {
        twoMotors.stop();
        return nextState;
    }
}
