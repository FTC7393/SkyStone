package org.firstinspires.ftc.teamcode.RoverRuckus;

import ftc.evlib.hardware.control.MecanumControl;
import ftc.evlib.hardware.sensors.Gyro;
import ftc.evlib.opmodes.AbstractAutoOp;
import ftc.evlib.statemachine.EVStateMachineBuilder;
import ftc.electronvolts.statemachine.StateMachine;
import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.util.TeamColor;
import ftc.electronvolts.util.files.Logger;
import ftc.electronvolts.util.units.Angle;
import ftc.electronvolts.util.units.Distance;
//@Autonomous(name = "RoverRuckusTestingOp")

public class RoverRuckusAutoStatess extends AbstractAutoOp<RoverRuckusRobotCfg> {
    Gyro gyro;
    MecanumControl mecanumControl;

    @Override
    protected RoverRuckusRobotCfg createRobotCfg() {
        return new RoverRuckusRobotCfg(hardwareMap);
    }

    @Override
    protected Logger createLogger() {
        return null;
    }


    @Override
    protected void setup_act() {

    }

    @Override
    protected void go() {

    }


    @Override
    protected void act() {

    }



    @Override
    protected void end() {

    }

    @Override
    public StateMachine buildStates() {
        TeamColor teamColor = TeamColor.RED;

        EVStateMachineBuilder b = robotCfg.createEVStateMachineBuilder(S.ANTITIP, teamColor, Angle.fromDegrees(1));
        b.addServo(S.ANTITIP, S.DRIVE_TO_DEPOT,robotCfg.backFoot.getName(),RoverRuckusRobotCfg.BackFootPresets.OPENED,false);
        b.addDrive(S.DRIVE_TO_DEPOT, S.RELEASE_MARKER, Distance.fromFeet(16),1,180,0);
        // Commented out next line when we updated the door code.
        //b.addServo(S.RELEASE_MARKER,S.STOP,robotCfg.Door.getName(),RoverRuckusRobotCfg.doorPresets.LEFT,true);

        b.addStop(S.STOP);

        return b.build();
    }


    private enum S implements StateName {
        LOCK_ROTATION,
        MOVE_OFF_HOOK,
        START,
        RELEASE_MARKER,
        ANTITIP,

        DETECT_GOLD,
        STOP,

        GOLD_ALIGN, DRIVE_TO_DEPOT, DRIVE_LITTLE, RELEASE_LATCH

    }
}
