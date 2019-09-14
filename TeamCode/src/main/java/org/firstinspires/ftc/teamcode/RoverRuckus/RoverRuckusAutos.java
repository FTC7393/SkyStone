package org.firstinspires.ftc.teamcode.RoverRuckus;//package org.firstinspires.ftc.teamcode.RoverRuckus;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.RoverRuckus.RoverRuckusTests.ConceptTensorFlowObjectDetection;
//
//import evlib.hardware.config.RobotCfg;
//import evlib.hardware.control.MecanumControl;
//import evlib.hardware.sensors.Gyro;
//import evlib.opmodes.AbstractAutoOp;
//import evlib.opmodes.AbstractOp;
//import evlib.statemachine.EVStateMachineBuilder;
//import ftc.electronvolts.statemachine.BasicAbstractState;
//import ftc.electronvolts.statemachine.StateMachine;
//import ftc.electronvolts.statemachine.StateName;
//import ftc.electronvolts.util.ResultReceiver;
//import ftc.electronvolts.util.TeamColor;
//import ftc.electronvolts.util.files.Logger;
//import ftc.electronvolts.util.units.Angle;
//import ftc.electronvolts.util.units.Distance;
//import ftc.electronvolts.util.units.Time;
////@Autonomous(name = "RoverRuckusAUtooOp")
//
//public class RoverRuckusAutos extends AbstractAutoOp<RoverRuckusRobotCfg> {
//    Gyro gyro;
//    MecanumControl mecanumControl;
//
//    @Override
//    protected RoverRuckusRobotCfg createRobotCfg() {
//        return new RoverRuckusRobotCfg(hardwareMap);
//    }
//
//    @Override
//    protected Logger createLogger() {
//        return null;
//    }
//
//
//    @Override
//    protected void setup_act() {
//
//    }
//
//    @Override
//    protected void go() {
//
//    }
//
//
//    @Override
//    protected void act() {
//
//    }
//
//
//
//    @Override
//    protected void end() {
//
//    }
//
//    @Override
//    public StateMachine buildStates() {
//        TeamColor teamColor = TeamColor.RED;
//
//        EVStateMachineBuilder b = robotCfg.createEVStateMachineBuilder(S.ANTITIP, teamColor, Angle.fromDegrees(1));
//        b.add(S.DETECT_GOLD, new BasicAbstractState() {
//            StateName goldPosition;
//            @Override
//            public void init() {
//
//            }
//
//            @Override
//            public boolean isDone() {
//                final ResultReceiver<Integer> goldLocationReceiver= ConceptTensorFlowObjectDetection.initThread(hardwareMap);
//
//
//                if(goldLocationReceiver.getValue()==1){
//                    goldPosition=S.GOLD_LEFT;
//
//
//
//                }
//                else if(goldLocationReceiver.getValue()==3){
//                    goldPosition=S.GOLD_RIGHT;
//
//
//
//                }
//                else if(goldLocationReceiver.getValue()==2){
//                    goldPosition=S.GOLD_CENTER;
//                }
//
//
//                return true;
//            }
//
//            @Override
//            public StateName getNextStateName() {
//                return goldPosition;
//            }
//        });
//        b.addDrive(S.GOLD_CENTER,S.DRIVE_TO_DEPOT,Distance.fromFeet(16),1,180,0);
//        b.addServo(S.ANTITIP,S.DRIVE_TO_DEPOT,robotCfg.backFoot.getName(),RoverRuckusRobotCfg.backFootPresets.OPENED,false);
//        b.addDrive(S.DRIVE_TO_DEPOT,S.ROTATE, Distance.fromFeet(16),1,180,0);
//        b.add(S.ROTATE, new BasicAbstractState() {
//            @Override
//            public void init() {
//
//            }
//
//            @Override
//            public boolean isDone() {
//                robotCfg.getArm().controlRotation(2);
//
//                return true;
//            }
//
//            @Override
//            public StateName getNextStateName() {
//                return S.EXTEND;
//            }
//        });
//        b.add(S.EXTEND, new BasicAbstractState() {
//            @Override
//            public void init() {
//
//            }
//
//            @Override
//            public boolean isDone() {
//                robotCfg.getArm().controlExtension(-2);
//                return true;
//            }
//
//            @Override
//            public StateName getNextStateName() {
//                return S.RELEASE_MARKER;
//            }
//        });
//        b.addServo(S.RELEASE_MARKER,S.STOP,robotCfg.Door.getName(),RoverRuckusRobotCfg.doorPresets.LEFT,true);
//
//        b.addStop(S.STOP);
//
//        return b.build();
//    }
//
//
//    private enum S implements StateName {
//        GOLD_CENTER,
//        GOLD_LEFT,
//        GOLD_RIGHT,
//        LOCK_ROTATION,
//        MOVE_OFF_HOOK,
//        START,
//        ROTATE,
//        RELEASE_MARKER,
//        EXTEND,
//        ANTITIP,
//        DETECT_GOLD,
//        STOP,
//
//        GOLD_ALIGN, DRIVE_TO_DEPOT, DRIVE_LITTLE, RELEASE_LATCH
//
//    }
//}
