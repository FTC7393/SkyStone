package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.google.common.collect.ImmutableList;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

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
import ftc.electronvolts.util.units.Distance;
import ftc.electronvolts.util.units.Time;

@Autonomous(name = "RoverRuckusAutoOpTest")
@Disabled
public class RoverRuckusAutoTest extends AbstractAutoOp<RoverRuckusRobotCfg> {
    Gyro gyro;
    MecanumControl mecanumControl;
    double orientationDepot;
    double directionDepot;
    Mineral left;
    Mineral right;
    Mineral middle;
    TeamColor teamColor;
    boolean isStartingDepot;
    boolean moveToOpponentCrater;
    double wait;

    GoldPosition goldPosition;
    final ResultReceiver<List<Mineral>> potentialMineralResultReceiver = new BasicResultReceiver<>();


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
        telemetry.addData("goldPosition",goldPosition);
    }





    @Override
    protected void end() {

    }

    @Override
    public StateMachine buildStates() {
        OptionsFile optionsFile = new OptionsFile(EVConverters.getInstance(), FileUtil.getOptionsFile(RoverRuckusOptionsOp.FILENAME));


         teamColor = TeamColor.RED;
         isStartingDepot=optionsFile.get(RoverRuckusOptionsOp.isStartingDepot, RoverRuckusOptionsOp.isStartingDepotDefault);
         moveToOpponentCrater=optionsFile.get(RoverRuckusOptionsOp.moveToOpponentCrater, RoverRuckusOptionsOp.moveToOpponentCraterDefault);
        if(!moveToOpponentCrater){
            orientationDepot=-45;
            directionDepot=-135;
        }
        else{
            orientationDepot=-135;
            directionDepot=-45;

        }
//        final ResultReceiver <GoldDetector.Detection> leftResultReceiver = new BasicResultReceiver<>();
//        final ResultReceiver <GoldDetector.Detection> middleResultReceiver = new BasicResultReceiver<>();
//        final ResultReceiver <GoldDetector.Detection> rightResultReceiver = new BasicResultReceiver<>();

//        final ResultReceiver<Boolean> notifyLeftResultReceiver=new BasicResultReceiver<>();
//        final ResultReceiver<Boolean> notifyMiddleResultReceiver=new BasicResultReceiver<>();
//        final ResultReceiver<Boolean> notifyRightResultReceiver=new BasicResultReceiver<>();

        final ResultReceiver <Mineral> mineralResultReceiver = new BasicResultReceiver<>();
        final ResultReceiver <Boolean> cameraActionNotifier = new BasicResultReceiver<>();


//        ObjectDetector.initThread(true, false, telemetry,hardwareMap,leftResultReceiver,notifyLeftResultReceiver) ;
//        ObjectDetector.initThread(false, false, telemetry,hardwareMap,middleResultReceiver,notifyMiddleResultReceiver) ;
//        ObjectDetector.initThread(false, true, telemetry,hardwareMap,rightResultReceiver,notifyRightResultReceiver) ;

        int numCycles = 3;
        ObjectDetectorTest.initThread(numCycles, telemetry,hardwareMap,mineralResultReceiver, cameraActionNotifier, potentialMineralResultReceiver);


        EVStateMachineBuilder b = robotCfg.createEVStateMachineBuilder(S.UP_HANGING, teamColor, Angle.fromDegrees(3));
        b.add(S.UP_HANGING, createDescendState(S.RELEASE_LATCH));
        //b.add(S.UNLATCH_SERVO)
        //b.add(S.DRIVE-90DEGREES)
        b.addServo(S.RELEASE_LATCH, S.DETECT_LEFT_GOLD, RoverRuckusRobotCfg.MainServoName.LATCH, RoverRuckusRobotCfg.LatchPresets.UNLATCH,true);

        b.add(S.DETECT_LEFT_GOLD, new BasicAbstractState() {
            @Override
            public void init() {
                cameraActionNotifier.setValue(true);

            }

            @Override
            public boolean isDone() {

                if (mineralResultReceiver.isReady()) {
                    left = mineralResultReceiver.getValue();
                    return true;
                }
                return false;
            }

            @Override
            public StateName getNextStateName() {
                return S.MOVE_SERVO_MIDDLE_GOLD;
            }
        });
        b.addServo(S.MOVE_SERVO_MIDDLE_GOLD, S.DETECT_MIDDLE_GOLD, RoverRuckusRobotCfg.MainServoName.PHONEPAN, RoverRuckusRobotCfg.PhonePanPresets.MIDDLE,1.0,true);
        b.add(S.DETECT_MIDDLE_GOLD, new BasicAbstractState() {
            @Override
            public void init() {
                cameraActionNotifier.setValue(true);
            }

            @Override
            public boolean isDone() {
                if (mineralResultReceiver.isReady()) {
                    middle = mineralResultReceiver.getValue();
                    return true;
                }
                return false;
            }

            @Override
            public StateName getNextStateName() {
                return S.MOVE_SERVO_RIGHT_GOLD;
            }
        });
        b.addServo(S.MOVE_SERVO_RIGHT_GOLD, S.DETECT_RIGHT_GOLD, RoverRuckusRobotCfg.MainServoName.PHONEPAN, RoverRuckusRobotCfg.PhonePanPresets.RIGHT,1.0,true);
        b.add(S.DETECT_RIGHT_GOLD, new BasicAbstractState() {
            @Override
            public void init() {
                cameraActionNotifier.setValue(true);
            }

            @Override
            public boolean isDone() {
                if (mineralResultReceiver.isReady()) {
                    right = mineralResultReceiver.getValue();
                    return true;
                }
                return false;
            }

            @Override
            public StateName getNextStateName() {
                return S.DRIVE_LITTLE;
            }
        });





        b.addDrive(S.DRIVE_LITTLE, S.CHOOSE_POSITION,Distance.fromFeet(.2),.5,270,0);

        b.add(S.CHOOSE_POSITION, new BasicAbstractState() {
            @Override
            public void init() {

            }

            @Override
            public boolean isDone() {
                return true;
            }

            @Override
            public StateName getNextStateName() {
                StateName postGoldStateName;

                GoldDetector.Detection ld = left.getType();
                GoldDetector.Detection md = middle.getType();
                GoldDetector.Detection rd = right.getType();
                GoldDetector.Detection G = GoldDetector.Detection.GOLD;
                GoldDetector.Detection SLVR = GoldDetector.Detection.SILVER;
                GoldDetector.Detection U = GoldDetector.Detection.NOTHING;

                List<GoldDetector.Detection> allDetections = ImmutableList.of(ld, md, rd);
                List<GoldPosition> allPositions = ImmutableList.of(GoldPosition.LEFT, GoldPosition.MIDDLE, GoldPosition.RIGHT);
                int n = allDetections.size();
                int numSilvers = 0;
                int numUnknowns = 0;
                goldPosition = null;
                GoldPosition unknownPosition = null;
                for (int i=0; i<n; i++) {
                    GoldDetector.Detection d = allDetections.get(i);
                    // use the last gold found
                    if (d == G) {
                        goldPosition = allPositions.get(i);
                    }
                    if (d == SLVR) {
                        numSilvers++;
                    }
                    if (d == U) {
                        unknownPosition = allPositions.get(i);
                    }
                }
                if (goldPosition == null) {
                    // There are no gold.
                    if (numSilvers == 2) {
                        // With no gols, and two silvers, choose the last must be unknown - choose that one.
                        goldPosition = unknownPosition;
                    }
                }
                if (goldPosition == null) {
                    goldPosition = GoldPosition.MIDDLE;
                }


                if(goldPosition== GoldPosition.LEFT){

                    postGoldStateName= S.LEFT_GOLD;
                }
                else if(goldPosition== GoldPosition.MIDDLE||goldPosition== GoldPosition.UNKNOWN){
                    postGoldStateName= S.MIDDLE_GOLD;



                }
                else{
                    //the only other option is right position
                    postGoldStateName= S.RIGHT_GOLD;

                }
                return postGoldStateName;
            }
        });


        //middle
//        b.add(S.ROTATE_UP, new BasicAbstractState() {
//            double extension=0;
//            double rotation=1;
//            boolean done=false;
//            @Override
//            public void init() {
//
//            }
//
//            @Override
//            public boolean isDone() {
//                while(!robotCfg.getArm().autoUse(extension,rotation)){
//                    done=false;
//
//                }
//                done =true;
//
//
//
//                return done;
//            }
//
//            @Override
//            public StateName getNextStateName() {
//                return S.STOP;
//            }
//        });
        b.addWait(S.MIDDLE_GOLD, S.STARTING_DEPOT_MIDDLE,0);
        b.add(S.STARTING_DEPOT_MIDDLE, new BasicAbstractState() {
            StateName depot;
            @Override
            public void init() {

            }

            @Override
            public boolean isDone() {
                return true;
            }

            @Override
            public StateName getNextStateName() {
                if(isStartingDepot){
                    depot= S.DRIVE_TO_DEPOT_MORE;


                }
                else{
                    depot= S.DRIVE_DEPOT_MIDDLE;
                }
                return depot;
            }
        });
//        b.addDrive(S.DRIVE_DEPOT_MIDDLE,S.CRATER_TURN,Distance.fromFeet(3),.5,270,270);
        b.addGyroTurn(S.DRIVE_DEPOT_MIDDLE, S.STRAFE_TO_CRATER,0, 0.5); // turn back to original dirn
        b.addDrive(S.STRAFE_TO_CRATER, S.STOP,Distance.fromFeet(1.5),.5,270,0);

        b.addDrive(S.DRIVE_TO_DEPOT_MORE, S.DRIVE_TO_DEPOT_BACK,Distance.fromFeet(5),.5,270,-90);
        b.addDrive(S.DRIVE_TO_DEPOT_BACK, S.TURN_CRATER_AGAIN,Distance.fromFeet(.25),.5,90,-90);



        //left
        b.addDrive(S.LEFT_GOLD, S.STARTING_DEPOT_LEFT,Distance.fromFeet(1.4),.5,-35,270);
        b.add(S.STARTING_DEPOT_LEFT, new BasicAbstractState() {
            StateName depot;
            @Override
            public void init() {

            }

            @Override
            public boolean isDone() {
                return true;
            }

            @Override
            public StateName getNextStateName() {
                if(isStartingDepot){
                    depot= S.LEFT_GOLD_FORWARD;


                }
                else{
                    depot= S.CRATER_TURN;
                }
                return depot;
            }
        });
        b.addGyroTurn(S.DEPOT_TURN_LEFT, S.STOP,-135,.5);

        b.addDrive(S.LEFT_GOLD_FORWARD, S.LEFT_GOLD_TO_MIDDLE,Distance.fromFeet(1.3),.5,270,270);
        b.addDrive(S.LEFT_GOLD_TO_MIDDLE, S.LEFT_GOLD_FORWARD_REMOVE,Distance.fromFeet(1.25),.5,215,270);
        b.addDrive(S.LEFT_GOLD_FORWARD_REMOVE, S.LEFT_GOLD_FORWARD_BACK,Distance.fromFeet(1),.5,270,270);
        b.addDrive(S.LEFT_GOLD_FORWARD_BACK, S.TURN_CRATER_AGAIN,Distance.fromFeet(1.2),.5,90,270);


        //right
        b.addDrive(S.RIGHT_GOLD, S.STARTING_DEPOT_RIGHT,Distance.fromFeet(1.4),.5,215,270);
        b.add(S.STARTING_DEPOT_RIGHT, new BasicAbstractState() {
            StateName depot;
            @Override
            public void init() {

            }

            @Override
            public boolean isDone() {
                return true;
            }

            @Override
            public StateName getNextStateName() {
                if(isStartingDepot){
                    depot= S.RIGHT_GOLD_FORWARD;


                }
                else{
                    depot= S.CRATER_TURN;
                }
                return depot;
            }
        });//-135
        b.addGyroTurn(S.CRATER_TURN, S.CRATER_DRIVE,-135,.5);
        b.addDrive(S.CRATER_DRIVE, S.STOP,Distance.fromFeet(1),.5,-135,-135);
        b.addDrive(S.RIGHT_GOLD_FORWARD, S.RIGHT_GOLD_TO_MIDDLE,Distance.fromFeet(1.3),.5,270,270);
        b.addDrive(S.RIGHT_GOLD_TO_MIDDLE, S.RIGHT_GOLD_FORWARD_REMOVE,Distance.fromFeet(1.25),.5,-35,270);
        b.addDrive(S.RIGHT_GOLD_FORWARD_REMOVE, S.RIGHT_GOLD_FORWARD_BACK,Distance.fromFeet(1.5),.5,270,270);
        b.addDrive(S.RIGHT_GOLD_FORWARD_BACK, S.TURN_CRATER_AGAIN,Distance.fromFeet(1.9),.5,90,270);




        b.addGyroTurn(S.TURN_CRATER_AGAIN, S.MOVE_LITTLE,orientationDepot,.5);

        //b.addWait(S.WAIT,S.DRIVE_TO_DEPOT,500);
        b.addDrive(S.MOVE_LITTLE, S.RELEASE_MARKER,Distance.fromInches(9),.5,directionDepot,orientationDepot);
        b.addServo(S.RELEASE_MARKER, S.WAIT_MARKER, RoverRuckusRobotCfg.MainServoName.MARKER, RoverRuckusRobotCfg.MarkerPresets.RELEASE,true);
        b.addWait(S.WAIT_MARKER, S.DRIVE_TO_CRATER,Time.fromSeconds(.5));

        //b.addWait(S.WAIT,S.DRIVE_TO_DEPOT,500);
        b.addDrive(S.DRIVE_TO_CRATER, S.STOP,Distance.fromFeet(8.4),.5,-directionDepot,orientationDepot);
        // to go towards the other crater is 135 degrees; note - the gain on the gyro on the gyro control needs adjusting to keep it from
        b.addStop(S.STOP);

        return b.build();
    }

    private State createDescendState(final StateName nextStateName) {
        return new BasicAbstractState() {
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
                return nextStateName;
            }
        };
    }


    private enum S implements StateName {
        WAIT,
        TURN_CRATER,
        LEFT_GOLD,
        RIGHT_GOLD,
        MIDDLE_GOLD,
        WAIT_MARKER,
        TURN_TO_DETECT,
        TURN_BACK,
        TURN_DETECTION,
        MOVE_LITTLE,
        MOVE_FROM_LANDER,
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
        GOLD_ALIGN,
        DRIVE_TO_DEPOT,
        DRIVE_LITTLE,
        DRIVE_TO_CRATER, RELEASE_LATCH
        ,LEFT_GOLD_FORWARD,
        LEFT_GOLD_TO_MIDDLE,
        RIGHT_GOLD_FORWARD,
        RIGHT_GOLD_TO_MIDDLE,
        CHOOSE_POSITION,
        DONT_HIT_GOLD_TURN,
        RIGHT_DONT_HIT_GOLD_TURN,
        COLLECT_MIDDLE,
        LEFT_GOLD_FORWARD_REMOVE, LEFT_GOLD_FORWARD_BACK, RIGHT_GOLD_FORWARD_REMOVE,
        RIGHT_GOLD_FORWARD_BACK, DRIVE_TO_DEPOT_BACK, STARTING_DEPOT, STARTING_DEPOT_RIGHT,
        STARTING_DEPOT_LEFT, DEPOT_TURN_RIGHT, DEPOT_TURN_LEFT, STARTING_DEPOT_MIDDLE,
        DRIVE_DEPOT_MIDDLE, STRAFE_TO_CRATER, DEPOT_TURN, CRATER_TURN, CRATER_DRIVE,
        //ROTATE_UP,
        DETECT_LEFT_GOLD, DETECT_RIGHT_GOLD, MOVE_SERVO_MIDDLE_GOLD, DETECT_MIDDLE_GOLD,
        MOVE_SERVO_LEFT_GOLD, MOVE_SERVO_RIGHT_GOLD, LEFT_DONT_HIT_GOLD_TURN


    }
}

