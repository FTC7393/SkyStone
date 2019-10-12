package org.firstinspires.ftc.teamcode.sRIBot;

import com.google.common.collect.ImmutableList;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoverRuckus.GoldDetector;
import org.firstinspires.ftc.teamcode.RoverRuckus.GoldPosition;
import org.firstinspires.ftc.teamcode.RoverRuckus.Mineral;
import org.firstinspires.ftc.teamcode.RoverRuckus.MineralDecisionMaker;
import org.firstinspires.ftc.teamcode.RoverRuckus.ObjectDetector;
import org.firstinspires.ftc.teamcode.RoverRuckus.RoverRuckusOptionsOp;
import org.firstinspires.ftc.teamcode.RoverRuckus.RoverRuckusRobotCfg;

import java.util.List;

import ftc.electronvolts.statemachine.BasicAbstractState;
import ftc.electronvolts.statemachine.State;
import ftc.electronvolts.statemachine.StateMachine;
import ftc.electronvolts.statemachine.StateMachineBuilder;
import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.util.BasicResultReceiver;
import ftc.electronvolts.util.InputExtractor;
import ftc.electronvolts.util.ResultReceiver;
import ftc.electronvolts.util.TeamColor;
import ftc.electronvolts.util.files.Logger;
import ftc.electronvolts.util.files.OptionsFile;
import ftc.electronvolts.util.units.Angle;
import ftc.electronvolts.util.units.Distance;
import ftc.electronvolts.util.units.Time;
import ftc.evlib.hardware.control.MecanumControl;
import ftc.evlib.hardware.motors.TwoMotors;
import ftc.evlib.hardware.sensors.Gyro;
import ftc.evlib.opmodes.AbstractAutoOp;
import ftc.evlib.statemachine.EVStateMachineBuilder;
import ftc.evlib.util.EVConverters;
import ftc.evlib.util.FileUtil;

@Autonomous(name = "SriSKS TankDriveAuto")

public class TankDriveAuto extends AbstractAutoOp<TankDriveRobotCfg> {


    @Override
    protected TankDriveRobotCfg createRobotCfg() {
        return new TankDriveRobotCfg(hardwareMap);
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
                        })).build()
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
        //!!!!!!!!!! CHANGE START CONDITION
//        StateName firstState;
        StateMachineBuilder b = new StateMachineBuilder(S.DRIVE);
        b.add(S.DRIVE, new MotorsOnState(robotCfg.getTwoMotors(), 0.8, S.WAIT));
        b.add(S.WAIT, new WaitState(300L, S.OFF));
        b.add(S.OFF, new MotorsOffState(robotCfg.getTwoMotors(), S.STOP));
        b.add(S.DRIVE, new MotorsOnState(robotCfg.getTwoMotors(), 0.8, S.WAIT));
        b.addStop(S.STOP);
        return b.build();
    }

    private enum S implements StateName {
        DRIVE, WAIT, OFF, STOP;
    }
}

class WaitState implements State {
    private final long WaitTimeMillis;
    private final StateName nextState;
    boolean isFirst = true;
    Long timeAtStart = 0L;

    public WaitState(long waitTimeMillis, StateName nextState) {
        WaitTimeMillis = waitTimeMillis;
        this.nextState = nextState;
    }

    @Override
    public StateName act() {
        if (isFirst) {
            timeAtStart = System.currentTimeMillis();
            isFirst = false;
            return null;
        }
        if (System.currentTimeMillis() - timeAtStart > WaitTimeMillis) {
            return nextState;
        } else {
            return null;
        }
    }
}

class MotorsOnState implements State {
    private final TwoMotors motors;
    private final double power;
    private final StateName nextState;

    public MotorsOnState(TwoMotors motors, double power, StateName nextState) {
        this.motors = motors;
        this.power = power;
        this.nextState = nextState;
    }


    @Override
    public StateName act() {
        motors.runMotors(power, power);
        return nextState;
    }
}

class MotorsOffState implements State {
    private final TwoMotors motors;
    private final StateName nextState;

    public MotorsOffState(TwoMotors motors, StateName nextState) {
        this.motors = motors;
        this.nextState = nextState;
    }

    @Override
    public StateName act() {
        motors.stop();
        return nextState;
    }
}



