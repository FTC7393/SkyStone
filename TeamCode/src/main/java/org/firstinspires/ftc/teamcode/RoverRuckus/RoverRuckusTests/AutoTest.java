package org.firstinspires.ftc.teamcode.RoverRuckus.RoverRuckusTests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RoverRuckus.RoverRuckusRobotCfg;

import evlib.hardware.control.MecanumControl;
import evlib.hardware.sensors.Gyro;
import evlib.opmodes.AbstractAutoOp;
import evlib.statemachine.EVStateMachineBuilder;
import ftc.electronvolts.statemachine.StateMachine;
import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.util.TeamColor;
import ftc.electronvolts.util.files.Logger;
import ftc.electronvolts.util.units.Angle;

@Autonomous(name = "RoverRuckusTestingAutoOp")

public class AutoTest extends AbstractAutoOp<RoverRuckusRobotCfg> {
    Gyro gyro;
    MecanumControl mecanumControl;

    @Override
    protected RoverRuckusRobotCfg createRobotCfg() {
        return new RoverRuckusRobotCfg(hardwareMap);
    }

    @Override
    protected Logger createLogger() {return null;
//        return new Logger("", "autoTest.csv", new ImmutableList.Builder<Logger.Column>()
//                .add(new Logger.Column("gyro", new InputExtractor<Double>() {
//                    @Override
//                    public Double getValue() {
//                        return gyro.getHeading();
//                    }
//                }))
//                .add(new Logger.Column("velocityR", new InputExtractor<Double>() {
//                    @Override
//                    public Double getValue() {
//                        return mecanumControl.getVelocityR();
//                    }
//                }))
//                .add(new Logger.Column("Motor 0", new InputExtractor<Double>() {
//                    @Override
//                    public Double getValue() {
//                        return mecanumControl.getMecanumMotors().getValue(0);
//                    }
//                }))
//                .add(new Logger.Column("Tolerance", new InputExtractor<Double>() {
//                    @Override
//                    public Double getValue() {
//                        return EVEndConditions.toleranceLog ;
//                    }
//                }  ))
//                .add(new Logger.Column("Separation", new InputExtractor<Double>() {
//                    @Override
//                    public Double getValue() {
//                        return EVEndConditions.separationLog ;
//                    }
//                })).build());


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

        EVStateMachineBuilder b = robotCfg.createEVStateMachineBuilder(S.TURN, teamColor, Angle.fromDegrees(3));
        b.addGyroTurn(S.TURN, S.STOP,150,.5);
//        b.addDrive(S.DRIVE,S.STOP,Distance.fromFeet(1),.5,10,0);
        b.addStop(S.STOP);

        return b.build();
    }


    private enum S implements StateName {
        TURN,
        DRIVE,
        WAIT,
        UP_HANGING,
        DOWN_HANGING,
        UNLATCH,
        LOCK_ROTATION,
        MOVE_OFF_HOOK,
        START,
        RELEASE_MARKER,
        ANTITIP,

        DETECT_GOLD,
        STOP,

        GOLD_ALIGN, DRIVE_TO_DEPOT, DRIVE_LITTLE,DRIVE_TO_CRATER, RELEASE_LATCH

    }
}
