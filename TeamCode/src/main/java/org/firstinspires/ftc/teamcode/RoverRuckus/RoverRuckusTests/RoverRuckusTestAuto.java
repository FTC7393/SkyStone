package org.firstinspires.ftc.teamcode.RoverRuckus.RoverRuckusTests;

import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;

import org.firstinspires.ftc.teamcode.RoverRuckus.RoverRuckusRobotCfg;
import org.firstinspires.ftc.teamcode.softwareTests.GoldAlignExample;
import org.firstinspires.ftc.teamcode.softwareTests.SamplingOrderExample;

import evlib.hardware.control.RotationControls;
import evlib.hardware.control.TranslationControls;
import evlib.opmodes.AbstractOp;
import ftc.electronvolts.util.ResultReceiver;
import ftc.electronvolts.util.files.Logger;
import ftc.electronvolts.util.units.Angle;
import ftc.electronvolts.util.units.Time;

/**
 * Created by ftc7393 on 10/13/2018.
 */

public class RoverRuckusTestAuto extends AbstractOp<RoverRuckusRobotCfg> {
    private StateName state = StateName.RELEASE_LATCH;
    Angle tolerance;
    Angle orientation;


    private enum StateName {
        START,

        DETECT_GOLD,


        GOLD_ALIGN, COLOR_SENSOR_ALIGN, GOLD_POSITION, RELEASE_LATCH

    }
    @Override
    protected RoverRuckusRobotCfg createRobotCfg() {
        return new RoverRuckusRobotCfg(hardwareMap);
    }

    @Override
    protected Logger createLogger() {
        return null;
    }

    @Override
    protected void setup() {

    }

    @Override
    protected void setup_act() {

    }

    @Override
    protected void go() {

    }

    @Override
    protected void pre_act() {

    }

    @Override
    protected void act() {
        final ResultReceiver<SamplingOrderDetector.GoldLocation> goldLocationReceiver= SamplingOrderExample.initThread(hardwareMap);

        SamplingOrderDetector.GoldLocation goldLocation;
        if(state== StateName.DETECT_GOLD){
            while(SamplingOrderDetector.GoldLocation.UNKNOWN==goldLocationReceiver.getValue()) {
                goldLocation = goldLocationReceiver.getValue();
            }
            state= StateName.COLOR_SENSOR_ALIGN;



        }

        if(state== StateName.COLOR_SENSOR_ALIGN){
            int redLeftColor=0;//robotCfg.getLeftColorSensor().red();
            int blueLeftColor=0;//robotCfg.getLeftColorSensor().blue();
            int redRightColor=0;//robotCfg.getRightColorSensor().red();
            int blueRightColor=0;//robotCfg.getRightColorSensor().blue();

            if(blueLeftColor>=2&&redLeftColor>2){


                robotCfg.getMecanumControl().setTranslationControl(TranslationControls.ZERO);
                robotCfg.getMecanumControl().setRotationControl(RotationControls.constant(.5));


            }

            else if(blueRightColor>=2&&redRightColor>=2){
                robotCfg.getMecanumControl().setTranslationControl(TranslationControls.ZERO);
                robotCfg.getMecanumControl().setRotationControl(RotationControls.constant(-.5));

            }
           else{
                robotCfg.getMecanumControl().setTranslationControl(TranslationControls.ZERO);
//                robotCfg.getMecanumControl().setRotationControl(RotationControls.gyro());

            }



        }
        if(state== StateName.GOLD_POSITION){
            if(SamplingOrderExample.order== SamplingOrderDetector.GoldLocation.LEFT){

            }
            else if(SamplingOrderExample.order== SamplingOrderDetector.GoldLocation.CENTER){

            }
            else if(SamplingOrderExample.order== SamplingOrderDetector.GoldLocation.RIGHT){

            }


        }
        if(state== StateName.GOLD_ALIGN) {
            while (GoldAlignExample.getXpos() < 250) {


            }
            while (GoldAlignExample.getXpos() > 350) {


            }

        }







    }

    @Override
    protected void post_act() {

    }

    @Override
    protected void end() {

    }

    @Override
    protected Time getMatchTime() {
        return null;
    }
}
