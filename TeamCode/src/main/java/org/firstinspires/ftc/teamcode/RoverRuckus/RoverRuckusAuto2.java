package org.firstinspires.ftc.teamcode.RoverRuckus;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import evlib.hardware.control.MecanumControl;
import evlib.hardware.sensors.Gyro;
import evlib.opmodes.AbstractAutoOp;
import evlib.statemachine.EVStateMachineBuilder;
import ftc.electronvolts.statemachine.BasicAbstractState;
import ftc.electronvolts.statemachine.StateMachine;
import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.util.TeamColor;
import ftc.electronvolts.util.files.Logger;
import ftc.electronvolts.util.units.Angle;
import ftc.electronvolts.util.units.Distance;

@Autonomous(name = "RoverRuckusAutoOp2")
@Disabled
public class RoverRuckusAuto2 extends AbstractAutoOp<RoverRuckusRobotCfg> {
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
        telemetry.addData("gyro", robotCfg.getGyro().getHeading());
        telemetry.addData("state", stateMachine.getCurrentStateName());
    }





    @Override
    protected void end() {

    }

    @Override
    public StateMachine buildStates() {
        TeamColor teamColor = TeamColor.RED;

        EVStateMachineBuilder b = robotCfg.createEVStateMachineBuilder(S.UP_HANGING, teamColor, Angle.fromDegrees(3));
        b.add(S.UP_HANGING, new BasicAbstractState() {
            private ElapsedTime runtime = new ElapsedTime();

            @Override
            public void init() {
                runtime.reset();

            }

            @Override
            public boolean isDone() {
                boolean doneHanging=false;
                if (!robotCfg.getHanging().islatchLimitPressed()&&runtime.seconds() < 6.0) {
//                        robotCfg.getMecanumControl().setRotationControl(RotationControls.ZERO);
//                        robotCfg.getMecanumControl().setTranslationControl(TranslationControls.constant(.2, Angle.fromDegrees(270)));

                    robotCfg.getHanging().upHanging();
                } else {
                    robotCfg.getHanging().stopHanging();
//                        robotCfg.getMecanumControl().setRotationControl(RotationControls.ZERO);
//                        robotCfg.getMecanumControl().setTranslationControl(TranslationControls.ZERO);
                    doneHanging=true;

                }


                return doneHanging;
            }

            @Override
            public StateName getNextStateName() {
                return S.RELEASE_LATCH;
            }
        });
        //b.add(S.UNLATCH_SERVO)
        //b.add(S.DRIVE-90DEGREES)
        b.addServo(S.RELEASE_LATCH, S.WAIT, RoverRuckusRobotCfg.MainServoName.LATCH, RoverRuckusRobotCfg.LatchPresets.UNLATCH,true);
        b.addWait(S.WAIT, S.DRIVE_TO_DEPOT,500);

        b.addDrive(S.DRIVE_TO_DEPOT, S.TURN_CRATER,Distance.fromFeet(.4),.5,270,0 );
        b.addGyroTurn(S.TURN_CRATER, S.DRIVE_TO_DEPOT_MORE,-45,.5);
        b.addDrive(S.DRIVE_TO_DEPOT_MORE, S.STOP,Distance.fromFeet(2),.5,270,-45);

        b.addStop(S.STOP);

        return b.build();
    }


    private enum S implements StateName {
        WAIT,
        TURN_CRATER,
        WAIT_MARKER,
        MOVE_LITTLE,
        TURN_CRATER_AGAIN,
        DRIVE_TO_DEPOT_MORE,
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

