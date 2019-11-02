package org.firstinspires.ftc.teamcode.RoverRuckus.Sandbox;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RoverRuckus.GoldPosition;
import org.firstinspires.ftc.teamcode.RoverRuckus.Mineral;
import org.firstinspires.ftc.teamcode.RoverRuckus.ObjectDetector;
import org.firstinspires.ftc.teamcode.RoverRuckus.RoverRuckusOptionsOp;
import org.firstinspires.ftc.teamcode.RoverRuckus.RoverRuckusRobotCfg;

import java.util.List;

import ftc.evlib.hardware.control.MecanumControl;
import ftc.evlib.hardware.sensors.Gyro;
import ftc.evlib.opmodes.AbstractAutoOp;
import ftc.evlib.statemachine.EVStateMachineBuilder;
import ftc.evlib.util.EVConverters;
import ftc.evlib.util.FileUtil;
import ftc.electronvolts.statemachine.BasicAbstractState;
import ftc.electronvolts.statemachine.State;
import ftc.electronvolts.statemachine.StateMachine;
import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.util.BasicResultReceiver;
import ftc.electronvolts.util.ResultReceiver;
import ftc.electronvolts.util.TeamColor;
import ftc.electronvolts.util.files.Logger;
import ftc.electronvolts.util.files.OptionsFile;
import ftc.electronvolts.util.units.Angle;
import ftc.electronvolts.util.units.Time;

@Autonomous(name = "Sandbox Auto 2")
public class SandboxAuto2 extends AbstractAutoOp<RoverRuckusRobotCfg> {
    Gyro gyro;
    MecanumControl mecanumControl;
    double orientationDepot;
    double directionDepot;
    ResultReceiver<Mineral> left;
    ResultReceiver<Mineral> middle;
    ResultReceiver<Mineral> right;
    TeamColor teamColor;
    boolean isStartingDepot;
    boolean moveToOpponentCrater;

    final ResultReceiver<List<Mineral>> potentialMineralResultReceiver = new BasicResultReceiver<>();

    GoldPosition goldPosition;


    @Override
    protected RoverRuckusRobotCfg createRobotCfg() {
        return new RoverRuckusRobotCfg(hardwareMap);
    }

    @Override
    protected Logger createLogger() {
        String prefix = "sandbox_log";
        String postfix = ".txt";
        return new Logger(prefix, postfix, robotCfg.getLoggerColumns());
    }

    @Override
    protected void setup_act() {

    }

    @Override
    protected void go() {

    }

    @Override
    protected void act() {
//        telemetry.addData("gyro ", robotCfg.getGyro().getHeading());
//        if (potentialMineralResultReceiver.isReady()) {
//            List<Mineral> mlist = potentialMineralResultReceiver.getValue();
//            for (Mineral m : mlist) {
//                m.showInTelem(telemetry);
//            }
//        }
        telemetry.addData("state", stateMachine.getCurrentStateName());
        Mineral lm = left.getValue();
        String ls="NA";
        if (lm != null) {
            ls = lm.getType() + " " + lm.getConfidence();
        }
        Mineral mm = middle.getValue();
        String ms="NA";
        if (mm != null) {
            ms = mm.getType() + " " + mm.getConfidence();
        }
        Mineral rm = right.getValue();
        String rs="NA";
        if (rm != null) {
            rs = rm.getType() + " " + rm.getConfidence();
        }
        telemetry.addData("LEFT mineral:", ls);
        telemetry.addData("MID  mineral:", ms);
        telemetry.addData("RGHT mineral:", rs);
        telemetry.addData("goldPosition",goldPosition);
    }

    @Override
    protected void end() {

    }
    enum S implements StateName {
        PAN_LEFT,
        PAUSE1,
        LOOK_FOR_GOLD_LEFT,
        PAN_MID,
        PAUSE2,
        LOOK_FOR_GOLD_MIDDLE,
        PAN_RIGHT,
        LOOK_FOR_GOLD_RIGHT,
        PAUSE3,
        STOP;
        }

    @Override
    public StateMachine buildStates() {
        OptionsFile optionsFile = new OptionsFile(EVConverters.getInstance(), FileUtil.getOptionsFile(RoverRuckusOptionsOp.FILENAME));


        teamColor = TeamColor.RED;
        isStartingDepot = optionsFile.get(RoverRuckusOptionsOp.isStartingDepot, RoverRuckusOptionsOp.isStartingDepotDefault);
        moveToOpponentCrater = optionsFile.get(RoverRuckusOptionsOp.moveToOpponentCrater, RoverRuckusOptionsOp.moveToOpponentCraterDefault);
        if (!moveToOpponentCrater) {
            orientationDepot = -45;
            directionDepot = -135;
        } else {
            orientationDepot = -135;
            directionDepot = -45;

        }

        left = new BasicResultReceiver<>();
        middle = new BasicResultReceiver<>();
        right = new BasicResultReceiver<>();


        final ResultReceiver<Mineral> mineralResultReceiver = new BasicResultReceiver<>();
        final ResultReceiver<Boolean> cameraActionNotifier = new BasicResultReceiver<>();
        int numCycles = 3;
        ObjectDetector.initThread(numCycles, telemetry, hardwareMap, mineralResultReceiver, cameraActionNotifier, potentialMineralResultReceiver);


        EVStateMachineBuilder b = robotCfg.createEVStateMachineBuilder(S.PAN_LEFT, teamColor, Angle.fromDegrees(3));
        RoverRuckusRobotCfg.MainServoName panServo = RoverRuckusRobotCfg.MainServoName.PHONEPAN;
        RoverRuckusRobotCfg.PhonePanPresets psLeft = RoverRuckusRobotCfg.PhonePanPresets.LEFT;
        RoverRuckusRobotCfg.PhonePanPresets psMid = RoverRuckusRobotCfg.PhonePanPresets.MIDDLE;
        RoverRuckusRobotCfg.PhonePanPresets psRight = RoverRuckusRobotCfg.PhonePanPresets.RIGHT;
        int pauseTime = 1;
        b.addServo(S.PAN_LEFT, S.PAUSE1, panServo, psLeft, 0.15, true);
        b.addWait(S.PAUSE1, S.LOOK_FOR_GOLD_LEFT, Time.fromSeconds(pauseTime));
        b.add(S.LOOK_FOR_GOLD_LEFT, getLookState(cameraActionNotifier, mineralResultReceiver, S.PAN_MID,left));
        b.addServo(S.PAN_MID, S.PAUSE2, panServo, psMid, 0.15, true);
        b.addWait(S.PAUSE2, S.LOOK_FOR_GOLD_MIDDLE, Time.fromSeconds(pauseTime));
        b.add(S.LOOK_FOR_GOLD_MIDDLE, getLookState(cameraActionNotifier, mineralResultReceiver, S.PAN_RIGHT,middle));
        b.addServo(S.PAN_RIGHT, S.PAUSE3, panServo, psRight, 0.15, true);
        b.addWait(S.PAUSE3, S.LOOK_FOR_GOLD_RIGHT, Time.fromSeconds(pauseTime));
        b.add(S.LOOK_FOR_GOLD_RIGHT, getLookState(cameraActionNotifier, mineralResultReceiver, S.STOP,right));
        b.addStop(S.STOP);
        return b.build();
    }

    private State getLookState(final ResultReceiver<Boolean> cameraActor, final ResultReceiver<Mineral> goldRR, final StateName next, final ResultReceiver<Mineral> posDetRR) {
        return new BasicAbstractState() {

            @Override
            public void init() {
                cameraActor.setValue(true);
            }

            @Override
            public boolean isDone() {
                if (goldRR.isReady()) {
                    posDetRR.setValue(goldRR.getValue());
                    goldRR.clear();
                    return true;
                }
                return false;
            }

            @Override
            public StateName getNextStateName() {
                return next;
            }
        };
    }

//    @Override
//    public StateMachine buildStates() {
//        FrameGrabber fg = null;
//        EVStateMachineBuilder b = new EVStateMachineBuilder(S.DRIVE1, TeamColor.RED, Angle.fromRadians(0.05),
//                robotCfg.getGyro(), fg, servos, robotCfg.getMecanumControl());
//        Distance dist;
//        b.addDrive(S.DRIVE1, S.TURN1, Distance.fromFeet(3), 0.75, Angle.fromDegrees(90), Angle.fromDegrees(0));
//        b.addGyroTurn(S.TURN1, S.STOP, 90,.1);
////        b.addDrive(S.DRIVE2, S.STOP, Distance.fromFeet(4), 0.35, Angle.fromDegrees(180), Angle.fromDegrees(180));
//
//        b.addMotorOn(S.M_ON, S.STOP, robotCfg.getTestMotor(), 0.8);
//        b.addWait(S.WAIT5, S.STOP, Time.fromSeconds(5));
//
//        b.addStop(S.STOP);
//        return b.build();
//    }


}
