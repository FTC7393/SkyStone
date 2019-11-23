package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import ftc.electronvolts.util.Function;
import ftc.electronvolts.util.Functions;
import ftc.electronvolts.util.Utility;
import ftc.evlib.opmodes.AbstractOptionsOp;


/**
 * Created by ftc7393 on 12/6/2017.
 */
@TeleOp(name = "OptionsOp")
@Disabled
public class RoverRuckusOptionsOp extends AbstractOptionsOp {

    public enum Opts {
        IS_STARTING_DEPOT("isStartingDepot"),
        MOVE_TO_OPPONENT_CRATER("moveToOpponentCrater"),
        DO_CAMERA_SAMPLING("doCameraSampling"),
        WAIT_TIME("wait"),
        DO_PARTNER_SAMPLE("doPartnerSample"),
        DO_CLAIM_CRATER_SIDE("doClaimCraterSide"),
        DESCEND("descend");

//        public boolean b;
//        public double f;
        public String s;

        Opts(String s) {
            this.s = s;
        }
    }
    private Opts[] values;
    int index = 0;
    /**
     * The filename will be set by the subclasses
     *
     * @param filename the name of the file where the options are stored
     */
    public RoverRuckusOptionsOp(String filename) {
        super(filename);
        values = Opts.values();
    }



    public static final String FILENAME = "options_rover_ruckus.txt";

//    public static final String teamColorTag = "teamColor";
    public static final String isStartingDepot="isStartingDepot";
    public static final String doCameraSampling="doCameraSampling";
    public static final String moveToOpponentCrater="moveToOpponentCrater";

//    public static final TeamColor teamColorDefault = TeamColor.RED;
    public static final boolean isStartingDepotDefault=false;
    public static final boolean moveToOpponentCraterDefault=true;
    public static final boolean doCameraSamplingDefault=true;
    public static final String wait="wait";
    public static final double waitDefault=0;
    public static final boolean claimFromCraterSideDefault = true;
    public static boolean doPartnerSampleDefault = false;
    public static boolean descendDefault = true;





    public RoverRuckusOptionsOp() {
        super(FILENAME);
        values = Opts.values();
    }

    @Override
    protected void go() {
        super.go();

    }

    @Override
    protected void act() {

//        if(driver1.right_stick_button.justPressed()){
//            optionsFile.set(isStartingDepot,true);
//        }
//        if(driver1.left_stick_button.justPressed()){
//            optionsFile.set(isStartingDepot,false);
//        }
//        telemetry.addData("left_stick_button=false,right_stick_button=true " + isStartingDepot, optionsFile.get(isStartingDepot, Boolean.class,null));
//
//
//        if(driver1.right_bumper.justPressed()){
//            optionsFile.set(moveToOpponentCrater,true);
//        }
//        if(driver1.left_bumper.justPressed()){
//            optionsFile.set(moveToOpponentCrater,false);
//        }
//        telemetry.addData("lb=false,rb=true " + moveToOpponentCrater, optionsFile.get(moveToOpponentCrater, Boolean.class,null));
//
//
//        if(driver1.x.justPressed()){
//            optionsFile.set(doCameraSampling,true);
//        }
//        if(driver1.y.justPressed()){
//            optionsFile.set(doCameraSampling,false);
//        }
//
//        telemetry.addData("y=false,x=true " + doCameraSampling, optionsFile.get(doCameraSampling, Boolean.class,null));
//
//
//        double waitTime = optionsFile.get(wait, waitDefault);
//
//        if (driver1.dpad_up.justPressed()) {
//            waitTime += .25;
//        }
//
//        if (driver1.dpad_down.justPressed()) {
//            waitTime -= .25;
//        }
//        waitTime = Utility.limit(waitTime, 0.00, 15.0);
//
//        optionsFile.set(wait, waitTime);
//        telemetry.addData("*1 dpad up/down => " + wait, String.format(Locale.ENGLISH, "%4.2f", waitTime));
        telemetry.addData("option",values[index]);
        if(driver1.x.justPressed()){
            index++;
            if(index>=values.length)
                index = 0;
        }
        if(driver1.b.justPressed()){
            index--;
            if(index<0){
                index = values.length-1;
            }
        }
        if(driver1.left_bumper.justPressed()){
            if(values[index] == Opts.WAIT_TIME) {
                double waitTime = optionsFile.get(wait, waitDefault);
                waitTime -=0.25;
                waitTime = Utility.limit(waitTime, 0.00, 15.0);
                optionsFile.set(Opts.WAIT_TIME.s,waitTime);
            }
            else {
                optionsFile.set(values[index].s, false);
            }
        }
        if(driver1.right_bumper.justPressed()) {
            if (values[index] == Opts.WAIT_TIME) {
                double waitTime = optionsFile.get(wait, waitDefault);
                waitTime += 0.25;
                waitTime = Utility.limit(waitTime, 0.00, 15.0);
                optionsFile.set(Opts.WAIT_TIME.s, waitTime);
            } else {
                optionsFile.set(values[index].s, true);
            }
        }
            if(values[index] == Opts.WAIT_TIME) {
                telemetry.addData("currentValue", optionsFile.get(values[index].s,waitDefault));
            }
            else {
                telemetry.addData("currentValue", optionsFile.get(values[index].s,false));
            }


    }

    public double pow10floor(double x) {
        return Math.pow(10, Math.floor(Math.log(x) / Math.log(10)));
    }

    @Override
    protected Function getJoystickScalingFunction() {
        return Functions.none();
    }

}
