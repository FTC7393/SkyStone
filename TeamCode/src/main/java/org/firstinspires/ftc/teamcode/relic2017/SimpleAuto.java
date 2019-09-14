package org.firstinspires.ftc.teamcode.relic2017;//package org.firstinspires.ftc.teamcode.relic2017;
//
//import com.google.common.collect.ImmutableList;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import ftc.electronvolts.statemachine.EndConditions;
//import ftc.electronvolts.statemachine.StateMachine;
//import ftc.electronvolts.statemachine.StateName;
//import ftc.electronvolts.util.InputExtractor;
//import ftc.electronvolts.util.TeamColor;
//import ftc.electronvolts.util.files.Logger;
//import ftc.electronvolts.util.files.OptionsFile;
//import ftc.electronvolts.util.units.Distance;
//import evlib.hardware.sensors.DigitalSensor;
//import evlib.hardware.sensors.SpikeDetector;
//import evlib.opmodes.AbstractAutoOp;
//import evlib.util.EVConverters;
//import evlib.util.FileUtil;
//
//import ftc.electronvolts.statemachine.BasicAbstractState;
//import ftc.electronvolts.statemachine.EndCondition;
//import ftc.electronvolts.util.BasicResultReceiver;
//import ftc.electronvolts.util.ResultReceiver;
//import ftc.electronvolts.util.units.Angle;
//import ftc.electronvolts.util.units.Time;
//import evlib.statemachine.EVStateMachineBuilder;
//
//import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
//
//
///**
// * Created by ftc7393 on 11/11/2017.
// */
////extends AbstractAutoOp<>
//@Autonomous(name = "Main Autonomous")
//
//public class SimpleAuto extends AbstractAutoOp<RobotCfg2017> {
//    private enum S implements StateName{
//
//        WAIT_FOR_GYRO_CALIBRATION,
//        SERVO_INIT,//servo initializing
//        START_BRANCH,
//        TEAM_COLOR_BRANCH,
//        STARTING_RELIC_BRANCH,
//        DETECT_VUMARK,
//        INIT_VUMARK,
//        VUMARK_LOCATION,
//
//
//
//        RED_START_BRANCH,
//
//
//        BLUE_START_BRANCH,
//
//        SENSOR_SERVO_DOWN,
//        SENSOR_SERVO_UP,
//
//        FIND_JEWEL_COLOR,
//        BLUE_JEWEL,
//        RED_JEWEL,
//        NO_JEWEL,
//        MOVE_JEWEL_LITTLE_UP,
//        SERVO_LOOP,
//        TURN_TO_BOX,
//        MOVE_TO_LINE,
//        HIT_LEFT,
//        HIT_RIGHT,
//        MOVE_CENTER,
//        MOVE_RIGHT,
//        GRAB_LEFT_BLOCK,
//        GRAB_RIGHT_BLOCK,
//        FIRST_LINE,
//        TIMOUT_LINE,
//
//        LEFT_SECOND_LINE,
//        RIGHT_SECOND_LINE,
//        MOVE_TILL_RIGHT, MOVE_TILL_LEFT, MOVE_TOWARD_LINE, CALIBRATE_LINE_SENSORS, TURN_LITTLE, WAIT_FOR_LINE_SENSORS, LIFT_LITTLE, MOVE_TO_MIDDLE, MOVE_BACK, RELEASE_SERVO, STOP, RELEASE_RIGHT_SERVO, RELEASE_LEFT_SERVO, LIFT_DOWN, MOVE_LEFT
//
//
//
//
//
//
//
//
//    }
//    private enum JewelColor {
//        RED_JEWEL,
//        BLUE_JEWEL,
//        NO_JEWEL
//    }
//
//    JewelColor jewelColor = JewelColor.NO_JEWEL;
//    private TeamColor teamColor;
//    private boolean isStartingRelic;
//
//
//
//    @Override
//    public StateMachine buildStates() {
//        OptionsFile optionsFile = new OptionsFile(EVConverters.getInstance(), FileUtil.getOptionsFile(OptionsOp.FILENAME));
//        final ResultReceiver<VuMarkFinder> vumarkReceiver= VuMarkFinder.initThread(hardwareMap);
//        final ResultReceiver <RelicRecoveryVuMark> vuMarkResultReceiver = new BasicResultReceiver<>();
//
//
//        teamColor = optionsFile.get(OptionsOp.teamColorTag,OptionsOp.teamColorDefault);
//        isStartingRelic=optionsFile.get(OptionsOp.isStartingRelic,OptionsOp.isStartingRelicDefault);
//        if (teamColor == TeamColor.UNKNOWN) {
//            throw new IllegalArgumentException("teamColor cannot be UNKNOWN when running autonomous!");
//        }
//        boolean isRed = teamColor == TeamColor.RED;
//        final SpikeDetector rightLineSensor= robotCfg.getRightLineSensor();
//        final SpikeDetector leftLineSensor= robotCfg.getLeftLineSensor();
//
//        final SpikeDetector intialLineSensor = isRed ? robotCfg.getRightLineSensor() : robotCfg.getLeftLineSensor();
//        final DigitalSensor otherLineSensor = isRed ? robotCfg.getRightLineSensor() : robotCfg.getLeftLineSensor();
//        double firstMoveOrientation;
//        double firstMoveDirection,secondMoveDirection,leftFirstThirdMoveDirection,rightFirstThirdMoveDirection,pushDirection;
//
//
//        if(isRed){
//
//            firstMoveDirection=-90;
//            if(isStartingRelic){
//                pushDirection=-180;
//
//            firstMoveOrientation=-180;
//           firstMoveDirection=firstMoveDirection;
//
//                secondMoveDirection=-90;
//
//
//            }
//            else{
//                firstMoveOrientation=-90;
//                secondMoveDirection=0;
//                pushDirection=90;
//
//
//            }
//        }
//        else{
//            firstMoveDirection=90;
//            if(isStartingRelic){
//                firstMoveOrientation=180;
//               firstMoveDirection=firstMoveDirection;
//                secondMoveDirection=90;
//                pushDirection=180;
//
//
//            }
//
//            else{
//                firstMoveOrientation=90;
//                secondMoveDirection=0;
//                pushDirection=-90;
//
//
//            }
//        }
//        firstMoveDirection=firstMoveDirection*-1;
//        secondMoveDirection=secondMoveDirection*-1;
//
//
//        double degreesTriangle=33.690067526;
//
//        leftFirstThirdMoveDirection=secondMoveDirection-degreesTriangle;
//        rightFirstThirdMoveDirection=secondMoveDirection+degreesTriangle;
//        leftFirstThirdMoveDirection=leftFirstThirdMoveDirection*-1;
//        rightFirstThirdMoveDirection=rightFirstThirdMoveDirection*-1;
//        EVStateMachineBuilder b = robotCfg.createEVStateMachineBuilder(S.SERVO_INIT, teamColor, Angle.fromDegrees(1));
//        b.addServoInit(S.SERVO_INIT,S.INIT_VUMARK);
//        b.addResultReceiverReady(S.INIT_VUMARK,S.DETECT_VUMARK,vumarkReceiver);
//
//        b.add(S.DETECT_VUMARK, new BasicAbstractState() {
//            EndCondition timeout= EndConditions.timed(Time.fromSeconds(2));
//
//            VuMarkFinder vumarkFinder;
//            RelicRecoveryVuMark vuMark;
//
//            @Override
//            public void init() {
//                timeout.init();
//                vumarkFinder =vumarkReceiver.getValue();
//                vumarkFinder.activate();
//
//            }
//
//            @Override
//            public boolean isDone() {
//
//
//                vumarkFinder.act();
//                vuMark =vumarkFinder.getVuMark();
//
//                return timeout.isDone()||vuMark != RelicRecoveryVuMark.UNKNOWN;
//            }
//
//
//            @Override
//            public StateName getNextStateName() {
//
//
//
//
//                return S.WAIT_FOR_GYRO_CALIBRATION;
//            }
//        });
//        b.addCalibrateGyro(S.WAIT_FOR_GYRO_CALIBRATION,S.GRAB_RIGHT_BLOCK);
//        b.addServo(S.GRAB_RIGHT_BLOCK,S.GRAB_LEFT_BLOCK,RobotCfg2017.MainServoName.RIGHTRELEASE,RobotCfg2017.RightReleaseServoPresets.GRAB,1.5,true);
//        b.addServo(S.GRAB_LEFT_BLOCK,S.LIFT_LITTLE,RobotCfg2017.MainServoName.LEFTRELEASE,RobotCfg2017.LeftReleaseServoPresets.GRAB,1.5,true);
//        b.add(S.LIFT_LITTLE, new BasicAbstractState() {
//            @Override
//            public void init() {
//                robotCfg.getGrabber().goLittleUp();
//            }
//
//            @Override
//            public boolean isDone() {
//                return true;
//            }
//
//            @Override
//            public StateName getNextStateName() {
//                return S.SENSOR_SERVO_DOWN;
//            }
//        });
//
//
//        b.addServo(S.SENSOR_SERVO_DOWN,S.FIND_JEWEL_COLOR,RobotCfg2017.MainServoName.SENSOR, RobotCfg2017.SensorServoPresets.DOWN,.7,true);
//
//
//        b.add(S.FIND_JEWEL_COLOR, new BasicAbstractState() {
//            int counter=0;
//
//            @Override
//            public void init() {
//                robotCfg.getColorSensor().enableLed(true);
//
//
//            }
//
//            @Override
//            public boolean isDone() {
//                counter++;
//                return counter>=10;
//            }
//
//            @Override
//            public StateName getNextStateName() {
//                telemetry.addData("Red  ", robotCfg.getColorSensor().red());
//                telemetry.addData("Blue", robotCfg.getColorSensor().blue());
//                StateName stateName;
//
//                int redColor=robotCfg.getColorSensor().red();
//                int blueColor=robotCfg.getColorSensor().blue();
//                int alpha=robotCfg.getColorSensor().alpha();
//                if(blueColor>=2){
//                    jewelColor=JewelColor.BLUE_JEWEL;
//                     stateName=S.BLUE_JEWEL;
//                }
//                else if(redColor>=2){
//                    jewelColor=JewelColor.RED_JEWEL;
//                    stateName=S.RED_JEWEL;
//
//                }
//                else{
//                    jewelColor=JewelColor.NO_JEWEL;
//                    stateName=S.NO_JEWEL;
//
//                }
//
//
//
//                return stateName ;
//            }
//        });
//
//        b.addCount(S.NO_JEWEL,S.MOVE_JEWEL_LITTLE_UP,S.SENSOR_SERVO_UP,2);
////        b.addGyroTurn(S.TURN_LITTLE,S.SENSOR_SERVO_UP,);
//        b.addServoAdd(S.MOVE_JEWEL_LITTLE_UP,S.FIND_JEWEL_COLOR,RobotCfg2017.MainServoName.SENSOR,-.025,.5,true);
//        b.addBranch(S.RED_JEWEL,S.HIT_RIGHT,S.HIT_LEFT,isRed);
//        b.addBranch(S.BLUE_JEWEL,S.HIT_LEFT,S.HIT_RIGHT,isRed);
//        b.addGyroTurn(S.HIT_LEFT,S.SENSOR_SERVO_UP,-10,.5);
//        b.addGyroTurn(S.HIT_RIGHT,S.SENSOR_SERVO_UP,10,.5);
//        b.addServo(S.SENSOR_SERVO_UP,S.MOVE_TOWARD_LINE,RobotCfg2017.MainServoName.SENSOR,RobotCfg2017.SensorServoPresets.UP,.8,true);
////        b.addDrive(S.MOVE_TOWARD_LINE, S.CALIBRATE_LINE_SENSORS, Distance.fromFeet(1),.6,firstMoveDirection,0);
//
//        b.addDrive(S.MOVE_TOWARD_LINE, S.TURN_TO_BOX, Distance.fromFeet(1.45),.6,firstMoveDirection,0);
//        b.addGyroTurn(S.TURN_TO_BOX,S.MOVE_TO_LINE,firstMoveOrientation);
////        b.addDrive(S.MOVE_BACK, S.RELEASE_RIGHT_SERVO, Distance.fromFeet(.8),.6,-firstMoveDirection,firstMoveOrientation);
////        b.add(S.LIFT_DOWN, new BasicAbstractState() {
////            @Override
////            public void init() {
////                robotCfg.getGrabber().goDown();
////            }
////
////            @Override
////            public boolean isDone() {
////                return true;
////            }
////
////            @Override
////            public StateName getNextStateName() {
////                return S.MOVE_TO_LINE;
////            }
////        });
//        b.addServo(S.RELEASE_RIGHT_SERVO,S.RELEASE_LEFT_SERVO,RobotCfg2017.MainServoName.RIGHTRELEASE,RobotCfg2017.RightReleaseServoPresets.HALT,false);
//        b.addServo(S.RELEASE_LEFT_SERVO,S.MOVE_BACK,RobotCfg2017.MainServoName.LEFTRELEASE,RobotCfg2017.LeftReleaseServoPresets.HALT,true);
//        b.addDrive(S.MOVE_TO_LINE, S.RELEASE_RIGHT_SERVO, Distance.fromInches(14),.6,pushDirection,firstMoveOrientation);
//        b.addDrive(S.MOVE_BACK, S.STOP, Distance.fromInches(5),19,pushDirection+180,firstMoveOrientation);
////        b.add(S.CALIBRATE_LINE_SENSORS, new BasicAbstractState() {
////            @Override
////            public void init() {
////                leftLineSensor.resetSensor();
////                rightLineSensor.resetSensor();
////
////            }r5r
////
////            @Override
////            public boolean isDone() {
////
////                return true;
////            }
////
////            @Override
////            public StateName getNextStateName() {
////                return S.TURN_TO_BOX;
////            }
////        });
//
//
//
////        b.add(S.WAIT_FOR_LINE_SENSORS, new BasicAbstractState() {
////            @Override
////            public void init() {
////
////
////            }
////
////            @Override
////            public boolean isDone() {
////
////                return leftLineSensor.isReady();
////            }
////
////            @Override
////            public StateName getNextStateName() {
////                return S.MOVE_TO_LINE;
////            }
////        });
//
////        b.addDrive(S.MOVE_TO_LINE, StateMap.of(
////                S.FIRST_LINE,EVEndConditions.digitalSensor(intialLineSensor)
////                ,S.TIMOUT_LINE,EVEndConditions.timed(Time.fromSeconds(5))
////
////        ),.1,firstMoveDirection,firstMoveOrientation);
////        b.addDrive(S.FIRST_LINE, StateMap.of(
////               // S.SECOND_LINE,EVEndConditions.any(ImmutableList.of(EVEndConditions.digitalSensor(intialLineSensor))),
////                S.LEFT_SECOND_LINE,EVEndConditions.digitalSensor(leftLineSensor),
////                S.RIGHT_SECOND_LINE,EVEndConditions.digitalSensor(rightLineSensor)
////                ,S.TIMOUT_LINE,EVEndConditions.timed(Time.fromSeconds(5))
////
////
////        ),.1,secondMoveDirection,firstMoveOrientation);
////        b.addDrive(S.LEFT_SECOND_LINE,StateMap.of(
////                S.VUMARK_LOCATION,EVEndConditions.digitalSensor(rightLineSensor)
////                ,S.TIMOUT_LINE,EVEndConditions.timed(Time.fromSeconds(5))
////
////
////        ),.1,-leftFirstThirdMoveDirection,firstMoveOrientation);
////        b.addDrive(S.RIGHT_SECOND_LINE,StateMap.of(
////                S.VUMARK_LOCATION,EVEndConditions.digitalSensor(leftLineSensor)
////                ,S.TIMOUT_LINE,EVEndConditions.timed(Time.fromSeconds(5))
////
////
////        ),.1,-rightFirstThirdMoveDirection,firstMoveOrientation);
//
//        RelicRecoveryVuMark vuMark = vuMarkResultReceiver.getValue();
//        double distance=0;
//        if(vuMark==RelicRecoveryVuMark.CENTER){
//            distance=1.7;
//        }
//        else if(vuMark==RelicRecoveryVuMark.LEFT){
//            distance=1.2;
//        }
//        else if(vuMark==RelicRecoveryVuMark.RIGHT){
//            distance=1.95;
//        }
//        else {
//            distance=1.2;
//        }
//
//
//
//        b.add(S.VUMARK_LOCATION, new BasicAbstractState() {
//            @Override
//            public void init() {
//
//            }
//
//            @Override
//            public boolean isDone() {
//                return true;
//            }
//
//            @Override
//            public StateName getNextStateName() {
//                RelicRecoveryVuMark vuMark = vuMarkResultReceiver.getValue();
//                StateName stateName;
//
//
//                if(vuMark==RelicRecoveryVuMark.CENTER){
//                    stateName=S.MOVE_CENTER;
//                }
//                else if(vuMark==RelicRecoveryVuMark.LEFT){
//                    stateName=S.MOVE_LEFT;
//                }
//                else if(vuMark==RelicRecoveryVuMark.RIGHT){
//                    stateName=S.MOVE_RIGHT;
//                }
//                else {
//                    stateName=S.MOVE_CENTER;
//                }
//
//                return stateName;
//            }
//        });
//
//
//
//
//        b.addStop(S.STOP);
//        b.addStop(S.MOVE_RIGHT);
//
//        b.addStop(S.MOVE_LEFT);
//        b.addStop(S.MOVE_CENTER);
//
//
//
//
//
//
//        return b.build();
//    }
//
//    @Override
//    protected RobotCfg2017 createRobotCfg() {
//        return new RobotCfg2017(hardwareMap);
//    }
//
//    @Override
//    protected Logger createLogger() {
//        //log the current state and sub-state, and all the sensors, motors, servos, etc. from MainRobotCfg
//        return new Logger("", "autonomous.csv", new ImmutableList.Builder<Logger.Column>().add(
//                new Logger.Column("state", new InputExtractor<String>() {
//                    @Override
//                    public String getValue() {
//
//
//
//                        return stateMachine.getCurrentStateName().name();
//                    }
//                })
//        ).addAll(robotCfg.getLoggerColumns()).build());
//    }
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
//    @Override
//    protected void act() {
////        robotCfg.getGyro().update();
//
//        telemetry.addData("State", stateMachine.getCurrentStateName());
//        telemetry.addData("leftServo",robotCfg.getServo(RobotCfg2017.MainServoName.LEFTRELEASE).isDone());
//        telemetry.addData("rightServo",robotCfg.getServo(RobotCfg2017.MainServoName.RIGHTRELEASE).isDone());
//        telemetry.addData("sensor" +"Servo",robotCfg.getServo(RobotCfg2017.MainServoName.SENSOR).isDone());
//        telemetry.addData("mode",robotCfg.getGrabber().getServo());
//        telemetry.addData("leftServoCurrent",robotCfg.getServo(RobotCfg2017.MainServoName.LEFTRELEASE).getCurrentPosition());
//        telemetry.addData("rightServoCurrent",robotCfg.getServo(RobotCfg2017.MainServoName.RIGHTRELEASE).getCurrentPosition());
//        telemetry.addData("leftServoTarget",robotCfg.getServo(RobotCfg2017.MainServoName.LEFTRELEASE).getTargetPosition());
//        telemetry.addData("rightServoTarget",robotCfg.getServo(RobotCfg2017.MainServoName.RIGHTRELEASE).getTargetPosition());
//        telemetry.addData("gyro",robotCfg.getGyro().getHeading());
//
//
//
//    }
//
//    @Override
//    protected void end() {
//
//    }
//}
