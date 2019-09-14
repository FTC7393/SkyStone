package org.firstinspires.ftc.teamcode.RoverRuckus.Sandbox;

import com.google.common.collect.ImmutableList;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RoverRuckus.GoldPosition;
import org.firstinspires.ftc.teamcode.RoverRuckus.Mineral;
import org.firstinspires.ftc.teamcode.RoverRuckus.ObjectDetectorTest;
import org.firstinspires.ftc.teamcode.RoverRuckus.RoverRuckusOptionsOp;
import org.firstinspires.ftc.teamcode.RoverRuckus.RoverRuckusRobotCfg;

import java.util.List;

import evlib.hardware.control.MecanumControl;
import evlib.hardware.sensors.Gyro;
import evlib.opmodes.AbstractAutoOp;
import evlib.statemachine.EVStateMachineBuilder;
import evlib.util.EVConverters;
import evlib.util.FileUtil;
import ftc.electronvolts.statemachine.BasicAbstractState;
import ftc.electronvolts.statemachine.StateMachine;
import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.util.BasicResultReceiver;
import ftc.electronvolts.util.MatchTimer;
import ftc.electronvolts.util.ResultReceiver;
import ftc.electronvolts.util.TeamColor;
import ftc.electronvolts.util.files.Logger;
import ftc.electronvolts.util.files.OptionsFile;
import ftc.electronvolts.util.units.Angle;

@Autonomous(name = "Sandbox Auto 3")
public class SandboxAuto3 extends AbstractAutoOp<RoverRuckusRobotCfg> {
    Gyro gyro;
    MecanumControl mecanumControl;

    GoldPosition goldPosition;
    final ResultReceiver<List<Mineral>> potentialMineralResultReceiver = new BasicResultReceiver<>();


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
//        telemetry.addData("gyro", robotCfg.getGyro().getHeading());
//        telemetry.addData("state", stateMachine.getCurrentStateName());
//        telemetry.addData("goldPosition",goldPosition);
    }

    @Override
    protected void end() {

    }
    enum S implements StateName {
        OBSERVE,
        STOP;
        }

    @Override
    public StateMachine buildStates() {
        OptionsFile optionsFile = new OptionsFile(EVConverters.getInstance(), FileUtil.getOptionsFile(RoverRuckusOptionsOp.FILENAME));


        TeamColor teamColor = TeamColor.RED;
        EVStateMachineBuilder b = robotCfg.createEVStateMachineBuilder(S.OBSERVE, teamColor, Angle.fromDegrees(3));
        final ObjectDetectorTest objDetector = new ObjectDetectorTest(hardwareMap, telemetry, potentialMineralResultReceiver);
        b.add(S.OBSERVE, new BasicAbstractState() {
            MatchTimer timer;
            @Override
            public void init() {
                objDetector.init();
                timer = new MatchTimer(1200000L);
            }

            @Override
            public boolean isDone() {
                if(timer.isMatchOver()) {
                    objDetector.finalize();
                    return true;
                }
                objDetector.act();
                return false;
            }

            @Override
            public StateName getNextStateName() {
                return S.STOP;
            }
        });
        b.addStop(S.STOP);
        return b.build();
    }

}
