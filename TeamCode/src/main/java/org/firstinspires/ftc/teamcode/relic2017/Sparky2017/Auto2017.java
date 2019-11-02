package org.firstinspires.ftc.teamcode.relic2017.Sparky2017;

import ftc.evlib.util.ImmutableList;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.relic2017.Mechanisms.VuMarkFinder;

import java.util.ArrayList;
import java.util.List;

import ftc.evlib.opmodes.AbstractAutoOp;
import ftc.evlib.statemachine.EVEndConditions;
import ftc.evlib.statemachine.EVStateMachineBuilder;
import ftc.evlib.util.EVConverters;
import ftc.evlib.util.FileUtil;
import ftc.electronvolts.statemachine.BasicAbstractState;
import ftc.electronvolts.statemachine.EndCondition;
import ftc.electronvolts.statemachine.EndConditions;
import ftc.electronvolts.statemachine.StateMachine;
import ftc.electronvolts.statemachine.StateMap;
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


/**
 * Created by ftc7393 on 11/11/2017.
 */
//extends AbstractAutoOp<>
@Autonomous(name = "Auto2017")
@Disabled
public class Auto2017 extends AbstractAutoOp<RobotCfg2017> {
    private enum S implements StateName{

        WAIT_FOR_GYRO_CALIBRATION,
        SERVO_INIT,//servo initializing
        START_BRANCH,
        TEAM_COLOR_BRANCH,
        STARTING_RELIC_BRANCH,
        DETECT_VUMARK,
        INIT_VUMARK,
        VUMARK_LOCATION,



        RED_START_BRANCH,


        BLUE_START_BRANCH,

        SENSOR_SERVO_DOWN,
        SENSOR_SERVO_UP,

        FIND_JEWEL_COLOR,
        BLUE_JEWEL,
        RED_JEWEL,
        NO_JEWEL,
        MOVE_JEWEL_LITTLE_UP,
        SERVO_LOOP,
        TURN_TO_BOX,
        MOVE_TO_LINE,
        HIT_LEFT,
        HIT_RIGHT,
        MOVE_CENTER,
        MOVE_RIGHT,
        GRAB_LEFT_BLOCK,
        GRAB_RIGHT_BLOCK,
        FIRST_LINE,
        TIMOUT_LINE,

        LEFT_SECOND_LINE,
        RIGHT_SECOND_LINE,
        MOVE_TILL_RIGHT, MOVE_TILL_LEFT, MOVE_TOWARD_LINE, CALIBRATE_LINE_SENSORS, TURN_LITTLE, WAIT_FOR_LINE_SENSORS, LIFT_LITTLE, MOVE_JEWEL_LITTLE_LEFT, MOVE_JEWEL_LITTLE_UP_COUNT, SENSOR_MIDDLE, MOVE_OFF_PLATFORM, RELEASE_RIGHT_SERVO, RELEASE_LEFT_SERVO, MOVE_BACK, STOP, RELEASE, RELEASER, STOP2, WAIT, SECOND_LINE, MOVE_TO_BOX, MOVE_TOWARD_BOX, LEFT_FIRST_THIRD_MOVE, RIGHT_FIRST_THIRD_MOVE, MOVE_LEFT








    }
    private enum JewelColor {
        RED_JEWEL,
        BLUE_JEWEL,
        NO_JEWEL
    }

    JewelColor jewelColor = JewelColor.NO_JEWEL;
    private TeamColor teamColor;
    private boolean isStartingRelic;

    ColoredLineEndCondition firstLine;
    ColoredLineEndCondition leftSecondLine;
    ColoredLineEndCondition rightSecondLine;
    ColoredLineEndCondition vumarkRightLocation;
    ColoredLineEndCondition vumarkLeftLocation;

    double firstMoveDirection,secondMoveDirection,leftFirstThirdMoveDirection,rightFirstThirdMoveDirection,pushDirection;






    @Override
    public StateMachine buildStates() {
        OptionsFile optionsFile = new OptionsFile(EVConverters.getInstance(), FileUtil.getOptionsFile(OptionsOp.FILENAME));
        final ResultReceiver<VuMarkFinder> vumarkReceiver= VuMarkFinder.initThread(hardwareMap);
        final ResultReceiver <RelicRecoveryVuMark> vuMarkResultReceiver = new BasicResultReceiver<>();


        teamColor = optionsFile.get(OptionsOp.teamColorTag,OptionsOp.teamColorDefault);
        isStartingRelic=optionsFile.get(OptionsOp.isStartingRelic,OptionsOp.isStartingRelicDefault);
        if (teamColor == TeamColor.UNKNOWN) {
            throw new IllegalArgumentException("teamColor cannot be UNKNOWN when running autonomous!");
        }
        boolean isRed = teamColor == TeamColor.RED;
//        final SpikeDetector rightLineSensor= robotCfg.getRightLineSensor();
//        final SpikeDetector leftLineSensor= robotCfg.getLeftLineSensor();
        final ColorSensor rightColorSensor = robotCfg.getRightColorSensor();
        final ColorSensor leftColorSensor = robotCfg.getLeftColorSensor();
        final ColorSensor intialColorSensor = isRed ?  robotCfg.getLeftColorSensor() : robotCfg.getRightColorSensor();
        final ColorSensor otherColorSensor = isRed ? robotCfg.getRightColorSensor() : robotCfg.getLeftColorSensor();


        double firstMoveOrientation;
//        double firstMoveDirection,secondMoveDirection,leftFirstThirdMoveDirection,rightFirstThirdMoveDirection,pushDirection;


        if(isRed){

            firstMoveDirection=-90;
            if(isStartingRelic){
                pushDirection=-180;

                firstMoveOrientation=-180;
                firstMoveDirection=firstMoveDirection+4;

                secondMoveDirection=-90;


            }
            else{
                firstMoveOrientation=-90;
                secondMoveDirection=0;
                pushDirection=90;


            }
        }
        else{
            firstMoveDirection=90;
            if(isStartingRelic){
                firstMoveOrientation=180;
                firstMoveDirection=firstMoveDirection+4;
                secondMoveDirection=90;
                pushDirection=180;


            }

            else{
                firstMoveOrientation=90;
                secondMoveDirection=0;
                pushDirection=-90;


            }
        }
        firstMoveDirection=firstMoveDirection*-1;
        secondMoveDirection=secondMoveDirection*-1;


        double degreesTriangle=33.690067526;

        leftFirstThirdMoveDirection=(secondMoveDirection+180)-degreesTriangle;
        rightFirstThirdMoveDirection=(secondMoveDirection+180)+degreesTriangle;
        leftFirstThirdMoveDirection=leftFirstThirdMoveDirection+180;//*-1
        rightFirstThirdMoveDirection=rightFirstThirdMoveDirection+180;
        leftFirstThirdMoveDirection= isRed ?   (483.690067526): (303.690067526);
        rightFirstThirdMoveDirection= isRed ?  236.309932474 : 416.309932474;

        EVStateMachineBuilder b = robotCfg.createEVStateMachineBuilder(S.SERVO_INIT, teamColor, Angle.fromDegrees(1));
        b.addServoInit(S.SERVO_INIT, S.INIT_VUMARK);
        b.addResultReceiverReady(S.INIT_VUMARK, S.DETECT_VUMARK,vumarkReceiver);

        b.add(S.DETECT_VUMARK, new BasicAbstractState() {
            EndCondition timeout= EndConditions.timed(Time.fromSeconds(2));

            VuMarkFinder vumarkFinder;
            RelicRecoveryVuMark vuMark;

            @Override
            public void init() {
                timeout.init();
                vumarkFinder =vumarkReceiver.getValue();
                vumarkFinder.activate();

            }

            @Override
            public boolean isDone() {


                vumarkFinder.act();
                vuMark =vumarkFinder.getVuMark();
                if(vuMark != RelicRecoveryVuMark.UNKNOWN) {
                    vuMarkResultReceiver.setValue(vuMark);

                }
                return timeout.isDone()||vuMark != RelicRecoveryVuMark.UNKNOWN;
            }


            @Override
            public StateName getNextStateName() {




                return S.WAIT_FOR_GYRO_CALIBRATION;
            }
        });
        b.addCalibrateGyro(S.WAIT_FOR_GYRO_CALIBRATION, S.LIFT_LITTLE);
//        b.addDrive(S.LEFT_FIRST_THIRD_MOVE,S.RIGHT_FIRST_THIRD_MOVE,Distance.fromFeet(1.5),.3,leftFirstThirdMoveDirection,0);
//        b.addDrive(S.RIGHT_FIRST_THIRD_MOVE,S.LIFT_LITTLE,Distance.fromFeet(1.5),.3,rightFirstThirdMoveDirection,0);

//        b.addServo(S.GRAB_RIGHT_BLOCK,S.GRAB_LEFT_BLOCK,RobotCfg2017.MainServoName.RIGHTRELEASE,RobotCfg2017.RightReleaseServoPresets.GRAB,1.5,true);
//        b.addServo(S.GRAB_LEFT_BLOCK,S.LIFT_LITTLE,RobotCfg2017.MainServoName.LEFTRELEASE,RobotCfg2017.LeftReleaseServoPresets.GRAB,1.5,true);
        b.add(S.LIFT_LITTLE, new BasicAbstractState() {
            @Override
            public void init() {
                robotCfg.getGrabber().goLittleUp();
            }

            @Override
            public boolean isDone() {
                return true;
            }

            @Override
            public StateName getNextStateName() {
                return S.SENSOR_SERVO_DOWN;
            }
        });


        b.addServo(S.SENSOR_SERVO_DOWN, S.FIND_JEWEL_COLOR,RobotCfg2017.MainServoName.SENSOR, RobotCfg2017.SensorServoPresets.DOWN,.7,true);


        b.add(S.FIND_JEWEL_COLOR, new BasicAbstractState() {
            int counter=0;

            @Override
            public void init() {
                robotCfg.getJewelColorSensor().enableLed(true);


            }

            @Override
            public boolean isDone() {
                counter++;
                return counter>=10;
            }

            @Override
            public StateName getNextStateName() {
                telemetry.addData("Red  ", robotCfg.getJewelColorSensor().red());
                telemetry.addData("Blue", robotCfg.getJewelColorSensor().blue());
                StateName stateName;

                int redColor=robotCfg.getJewelColorSensor().red();
                int blueColor=robotCfg.getJewelColorSensor().blue();
                int alpha=robotCfg.getJewelColorSensor().alpha();
                if(blueColor>=2){
                    jewelColor= JewelColor.BLUE_JEWEL;
                    stateName= S.BLUE_JEWEL;
                }
                else if(redColor>=2){
                    jewelColor= JewelColor.RED_JEWEL;
                    stateName= S.RED_JEWEL;

                }
                else{
                    jewelColor= JewelColor.NO_JEWEL;
                    stateName= S.NO_JEWEL;

                }



                return stateName ;
            }
        });
        b.addCount(S.NO_JEWEL, S.MOVE_JEWEL_LITTLE_LEFT, S.MOVE_JEWEL_LITTLE_UP_COUNT,7);

        b.addCount(S.MOVE_JEWEL_LITTLE_UP_COUNT, S.MOVE_JEWEL_LITTLE_UP, S.SENSOR_MIDDLE,1);

//        b.addGyroTurn(S.TURN_LITTLE,S.SENSOR_SERVO_UP,);
        b.addServoAdd(S.MOVE_JEWEL_LITTLE_UP, S.FIND_JEWEL_COLOR,RobotCfg2017.MainServoName.SENSOR,-.05,.5,true);
        b.addServoAdd(S.MOVE_JEWEL_LITTLE_LEFT, S.FIND_JEWEL_COLOR,RobotCfg2017.MainServoName.JEWEL,-.025,.5,true);

        b.addBranch(S.RED_JEWEL, S.HIT_RIGHT, S.HIT_LEFT,isRed);
        b.addBranch(S.BLUE_JEWEL, S.HIT_LEFT, S.HIT_RIGHT,isRed);
        b.addServo(S.HIT_LEFT, S.SENSOR_SERVO_UP,RobotCfg2017.MainServoName.JEWEL,RobotCfg2017.jewelServoPresets.LEFT,true);
        b.addServo(S.HIT_RIGHT, S.SENSOR_SERVO_UP,RobotCfg2017.MainServoName.JEWEL,RobotCfg2017.jewelServoPresets.RIGHT,true);
//        b.addServo(S.SENSOR_MIDDLE,S.SENSOR_SERVO_UP,RobotCfg2017.MainServoName.JEWEL,RobotCfg2017.jewelServoPresets.MIDDLE,true);

        b.addServo(S.SENSOR_SERVO_UP, S.MOVE_TOWARD_LINE,RobotCfg2017.MainServoName.SENSOR,RobotCfg2017.SensorServoPresets.UP,.8,true);
//        b.addDrive(S.MOVE_TOWARD_LINE, S.CALIBRATE_LINE_SENSORS, Distance.fromFeet(1),.6,firstMoveDirection,0);

        b.addDrive(S.MOVE_TOWARD_LINE, S.TURN_TO_BOX, Distance.fromFeet(.95),.3,firstMoveDirection,0);
        b.addGyroTurn(S.TURN_TO_BOX, S.MOVE_TO_LINE,firstMoveOrientation);

        firstLine = new ColoredLineEndCondition(intialColorSensor);
        leftSecondLine = new ColoredLineEndCondition(leftColorSensor);
        rightSecondLine = new ColoredLineEndCondition(rightColorSensor);
        vumarkRightLocation = new ColoredLineEndCondition(rightColorSensor);
        vumarkLeftLocation = new ColoredLineEndCondition(leftColorSensor);

        b.addDrive(S.MOVE_TO_LINE, StateMap.of(
                S.FIRST_LINE,firstLine
                , S.TIMOUT_LINE,EVEndConditions.timed(Time.fromSeconds(5))

        ),.1,firstMoveDirection,firstMoveOrientation);
        b.addDrive(S.FIRST_LINE, S.SECOND_LINE,Distance.fromInches(3),.6,firstMoveDirection,firstMoveOrientation);
        b.addDrive(S.SECOND_LINE, StateMap.of(
               // S.SECOND_LINE,EVEndConditions.any(ImmutableList.of(EVEndConditions.digitalSensor(intialLineSensor))),
                S.LEFT_SECOND_LINE,leftSecondLine,
                S.RIGHT_SECOND_LINE,rightSecondLine
                , S.TIMOUT_LINE,EVEndConditions.timed(Time.fromSeconds(5))


        ),.1,secondMoveDirection,firstMoveOrientation);
        b.addDrive(S.LEFT_SECOND_LINE,StateMap.of(
                S.VUMARK_LOCATION,vumarkRightLocation
                , S.TIMOUT_LINE,EVEndConditions.timed(Time.fromSeconds(5))


        ),.1,leftFirstThirdMoveDirection,firstMoveOrientation);
        b.addDrive(S.RIGHT_SECOND_LINE,StateMap.of(
                S.VUMARK_LOCATION,vumarkLeftLocation
                , S.TIMOUT_LINE,EVEndConditions.timed(Time.fromSeconds(5))


        ),.1,rightFirstThirdMoveDirection,firstMoveOrientation);
//
//        b.addDrive(S.MOVE_TO_LINE, S.RELEASE, Distance.fromInches(14),.6,pushDirection,firstMoveOrientation);
//
//        b.addServo(S.RELEASE,S.RELEASER,RobotCfg2017.MainServoName.LEFTRELEASE,RobotCfg2017.LeftReleaseServoPresets.EJECT,false);
//        b.addServo(S.RELEASER,S.WAIT,RobotCfg2017.MainServoName.RIGHTRELEASE,RobotCfg2017.RightReleaseServoPresets.EJECT,false);
//        b.addWait(S.WAIT,S.MOVE_BACK,4000);
//        b.addDrive(S.MOVE_BACK, S.STOP, Distance.fromInches(12),.3,pushDirection+185,firstMoveOrientation);
//        b.addDrive(S.STOP,S.STOP2,Distance.fromFeet(1),.6,pushDirection+185,firstMoveOrientation);





        b.add(S.VUMARK_LOCATION, new BasicAbstractState() {

            @Override
            public void init() {

            }

            @Override
            public boolean isDone() {
                return true;
            }

            @Override
            public StateName getNextStateName() {
                RelicRecoveryVuMark vuMark = vuMarkResultReceiver.getValue();
                StateName stateName;


                if(vuMark== RelicRecoveryVuMark.CENTER){

                    stateName= S.MOVE_CENTER;
                }
                else if(vuMark== RelicRecoveryVuMark.LEFT){

                    stateName= S.MOVE_LEFT;
                }
                else if(vuMark== RelicRecoveryVuMark.RIGHT){

                    stateName= S.MOVE_RIGHT;
                }
                else {
                    stateName= S.MOVE_RIGHT;
                }

                return stateName;
            }
        });
        b.addDrive(S.MOVE_CENTER, S.MOVE_TOWARD_BOX,Distance.fromInches(0),.3,secondMoveDirection,firstMoveOrientation);
        b.addDrive(S.MOVE_RIGHT, S.MOVE_TOWARD_BOX,Distance.fromInches(4.75),.3,secondMoveDirection+180,firstMoveOrientation);
        b.addDrive(S.MOVE_LEFT, S.MOVE_TOWARD_BOX,Distance.fromInches(5.5),.3,secondMoveDirection,firstMoveOrientation);

        b.addDrive(S.MOVE_TOWARD_BOX, S.RELEASE,Distance.fromInches(11),.3,pushDirection,firstMoveOrientation);
        b.addServo(S.RELEASE, S.RELEASER,RobotCfg2017.MainServoName.BOTTOMLEFTRELEASE,RobotCfg2017.BottomLeftReleaseServoPresets.EJECT,false);
        b.addServo(S.RELEASER, S.WAIT,RobotCfg2017.MainServoName.BOTTOMRIGHTRELEASE,RobotCfg2017.BottomRightReleaseServoPresets.EJECT,false);
        b.addWait(S.WAIT, S.MOVE_BACK,3000);
        b.addDrive(S.MOVE_BACK, S.STOP, Distance.fromInches(9),.3,pushDirection+185,firstMoveOrientation);
        b.addDrive(S.STOP, S.STOP2,Distance.fromFeet(1),.6,pushDirection+185,firstMoveOrientation);




        b.addStop(S.STOP2);//used to be: RELEASE
//        b.addStop(S.LEFT_SECOND_LINE);
//        b.addStop(S.RIGHT_SECOND_LINE);

//        b.addStop(S.MOVE_LEFT);
//        b.addStop(S.MOVE_CENTER);






        return b.build();
    }

    @Override
    protected RobotCfg2017 createRobotCfg() {
        return new RobotCfg2017(hardwareMap);
    }

    @Override
    protected Logger createLogger() {
        //log the current state and sub-state, and all the sensors, motors, servos, etc. from MainRobotCfg
        List<Logger.Column> cols = new ArrayList<>();
        cols.add(new Logger.Column("state", new InputExtractor<String>() {
                    @Override
                    public String getValue() {
                        return stateMachine.getCurrentStateName().name();
                    }
                })
        );
        cols.addAll(robotCfg.getLoggerColumns());

        return new Logger("", "autonomous.csv", cols);
    }

    @Override
    protected void setup_act() {

    }

    @Override
    protected void go() {

    }

    @Override
    protected void act() {
//        robotCfg.getGyro().update();

        telemetry.addData("State", stateMachine.getCurrentStateName());
//        telemetry.addData("leftServo",robotCfg.getServo(RobotCfg2017.MainServoName.LEFTRELEASE).isDone());
//        telemetry.addData("rightServo",robotCfg.getServo(RobotCfg2017.MainServoName.RIGHTRELEASE).isDone());
        telemetry.addData("sensor" +"Servo",robotCfg.getServo(RobotCfg2017.MainServoName.SENSOR).isDone());
//        telemetry.addData("mode",robotCfg.getGrabber().getServo());

//        telemetry.addData("leftServoCurrent",robotCfg.getServo(RobotCfg2017.MainServoName.LEFTRELEASE).getCurrentPosition());
//        telemetry.addData("rightServoCurrent",robotCfg.getServo(RobotCfg2017.MainServoName.RIGHTRELEASE).getCurrentPosition());
//        telemetry.addData("leftServoTarget",robotCfg.getServo(RobotCfg2017.MainServoName.LEFTRELEASE).getTargetPosition());
//        telemetry.addData("rightServoTarget",robotCfg.getServo(RobotCfg2017.MainServoName.RIGHTRELEASE).getTargetPosition());
        telemetry.addData("gyro",robotCfg.getGyro().getHeading());
        telemetry.addData("hue",firstLine.getValue());
        telemetry.addData("firstLeft",leftFirstThirdMoveDirection);
        telemetry.addData("firstRight",rightFirstThirdMoveDirection);






    }

    @Override
    protected void end() {

    }
}
