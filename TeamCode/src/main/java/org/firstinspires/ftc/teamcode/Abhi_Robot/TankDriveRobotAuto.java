package org.firstinspires.ftc.teamcode.Abhi_Robot;

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

@Autonomous(name = "TankDriveAuto")

public class TankDriveRobotAuto extends AbstractAutoOp<TankDriveRobotCfg> {


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
        b.add(S.DRIVE, new MotorsOnState(robotCfg.getTwoMotors(), 0.8, S.WAIT, 3000));
        b.add(S.WAIT, new waitState(S.OFF, 3000));
        b.add(S.OFF, new StopMotors(robotCfg.getTwoMotors(), S.STOP));
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

class MotorsOnState implements State {

    private final TwoMotors twoMotors;
    private final double power;
    private final StateName nextState;
    private final long runTimeMillis;
    private boolean isFirst = true;
    private long timeAtStart;


    public MotorsOnState(TwoMotors twoMotors, double power, StateName nextState, long runTimeMillis) {
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
