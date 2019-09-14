package org.firstinspires.ftc.teamcode.RoverRuckus.Sandbox;

import com.google.common.collect.ImmutableList;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RoverRuckus.RoverRuckusRobotCfg;

import evlib.opmodes.AbstractAutoOp;
import evlib.statemachine.EVStateMachineBuilder;
import evlib.vision.framegrabber.FrameGrabber;
import ftc.electronvolts.statemachine.StateMachine;
import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.util.TeamColor;
import ftc.electronvolts.util.files.Logger;
import ftc.electronvolts.util.units.Angle;
import ftc.electronvolts.util.units.Distance;
import ftc.electronvolts.util.units.Time;

@Autonomous(name = "Sandbox Auto")
public class SandboxAuto extends AbstractAutoOp<RoverRuckusRobotCfg> {

    enum S implements StateName {
        DRIVE1,
        TURN1,
//        DRIVE2,
        M_ON,
        WAIT5,
        STOP;
    }

    @Override
    public StateMachine buildStates() {
        FrameGrabber fg = null;
        EVStateMachineBuilder b = new EVStateMachineBuilder(S.DRIVE1, TeamColor.RED, Angle.fromRadians(0.05),
                robotCfg.getGyro(), fg, servos, robotCfg.getMecanumControl());
        Distance dist;
        b.addDrive(S.DRIVE1, S.STOP,Distance.fromFeet(.5),.5,270,0);

        // b.addDrive(S.DRIVE1, S.TURN1, Distance.fromFeet(3), 0.75, Angle.fromDegrees(90), Angle.fromDegrees(0));
        b.addGyroTurn(S.TURN1, S.STOP, 90,.1);
//        b.addDrive(S.DRIVE2, S.STOP, Distance.fromFeet(4), 0.35, Angle.fromDegrees(180), Angle.fromDegrees(180));

        b.addMotorOn(S.M_ON, S.STOP, robotCfg.getTestMotor(), 0.8);
        b.addWait(S.WAIT5, S.STOP, Time.fromSeconds(5));

        b.addStop(S.STOP);
        return b.build();
    }

    @Override
    protected RoverRuckusRobotCfg createRobotCfg() {
        return new RoverRuckusRobotCfg(hardwareMap);
    }

    @Override
    protected Logger createLogger() {
        String prefix = "sandbox_log";
        String postfix = ".txt";
        ImmutableList.Builder<Logger.Column> cols = ImmutableList.builder();
        cols.addAll(robotCfg.getLoggerColumns());
        return new Logger(prefix, postfix, cols.build());
    }

    @Override
    protected void setup_act() {

    }

    @Override
    protected void go() {

    }

    @Override
    protected void act() {
        telemetry.addData("gyro", robotCfg.getGyro().getHeading());
        telemetry.addData("state", stateMachine.getCurrentStateName());
    }

    @Override
    protected void end() {

    }

}
