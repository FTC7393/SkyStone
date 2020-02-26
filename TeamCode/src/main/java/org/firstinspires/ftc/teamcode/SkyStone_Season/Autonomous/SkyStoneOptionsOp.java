package org.firstinspires.ftc.teamcode.SkyStone_Season.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import ftc.electronvolts.util.Function;
import ftc.electronvolts.util.Functions;
import ftc.electronvolts.util.TeamColor;
import ftc.electronvolts.util.Utility;
import ftc.evlib.opmodes.AbstractOptionsOp;


/**
 * Created by ftc7393 on 12/6/2017.
 */
@TeleOp(name = "SkyStone OptionsOp")
public class SkyStoneOptionsOp extends AbstractOptionsOp {

    public static final String FILENAME = "options_skystone.txt";
    public static final String teamColorTag = "teamColor";
    public static final TeamColor teamColorDefault = TeamColor.BLUE;
    public static final String doSkyStoneTag = "doSkyStone";
    private static String verticalSlideCalTag = "verticalSlideCal";
    public static double offsetDefault = 0;
    private int index = 0;
    private Opts[] values;
    public static boolean doSkyStoneDefault = true;
    double offset;

    /**
     * The filename will be set by the subclasses
     *
     * @param filename the name of the file where the options are stored
     */
    public SkyStoneOptionsOp(String filename) {
        super(filename);
        values = Opts.values();
    }


    public SkyStoneOptionsOp() {
        super(FILENAME);
        values = Opts.values();
    }

    @Override
    protected void go() {
        super.go();
        offset = optionsFile.get(Opts.VERTICAL_SLIDE_CALIBRATION.s, offsetDefault);
    }

    @Override
    protected void act() {

        telemetry.addData("option", values[index]);
        if (driver1.x.justPressed()) {
            index++;
            if (index >= values.length)
                index = 0;
        }
        if (driver1.b.justPressed()) {
            index--;
            if (index < 0) {
                index = values.length - 1;
            }
        }

        if (values[index] == Opts.TEAM_COLOR) {
            if (driver1.right_bumper.justPressed()) {
                TeamColor teamColor = optionsFile.get(Opts.TEAM_COLOR.s, teamColorDefault);
                if (teamColor == TeamColor.BLUE) {
                    teamColor = TeamColor.RED;
                } else {
                    teamColor = TeamColor.BLUE;
                }
                optionsFile.set(Opts.TEAM_COLOR.s, teamColor);
                saveOptionsFile();
            }
        }


        if (values[index] == Opts.DO_SKYSTONE) {
            if (driver1.right_bumper.justPressed()) {
                boolean doSkyStone = optionsFile.get(Opts.DO_SKYSTONE.s, doSkyStoneDefault);
                if (doSkyStone) {
                    doSkyStone = false;
                } else {
                    doSkyStone = true;
                }
                optionsFile.set(Opts.DO_SKYSTONE.s, doSkyStone);
                saveOptionsFile();
            }
        }


        if (values[index] == Opts.VERTICAL_SLIDE_CALIBRATION) {
            offset += driver1.left_stick_y.getValue();
            offset = Utility.limit(offset, -7000, 7000);
            if (driver1.right_bumper.justPressed()) {
                optionsFile.set(Opts.VERTICAL_SLIDE_CALIBRATION.s, offset);
                saveOptionsFile();
            }
        }
//        if(driver1.right_bumper.justPressed()) {
//            if (values[index] == Opts.WAIT_TIME) {
//                double waitTime = optionsFile.get(wait, waitDefault);
//                waitTime += 0.25;
//                waitTime = Utility.limit(waitTime, 0.00, 15.0);
//                optionsFile.set(Opts.WAIT_TIME.s, waitTime);
//            } else {
//                optionsFile.set(values[index].s, true);
//            }
//        }
//            if(values[index] == Opts.WAIT_TIME) {
//                telemetry.addData("currentValue", optionsFile.get(values[index].s,waitDefault));
//            }
//            else {
//                telemetry.addData("currentValue", optionsFile.get(values[index].s,false));
//            }

        telemetry.addData(teamColorTag, optionsFile.get(Opts.TEAM_COLOR.s, teamColorDefault));


        telemetry.addData(doSkyStoneTag, optionsFile.get(Opts.DO_SKYSTONE.s, doSkyStoneDefault));

        telemetry.addData(verticalSlideCalTag, optionsFile.get(Opts.VERTICAL_SLIDE_CALIBRATION.s, offsetDefault));

        telemetry.addData("NEW " + verticalSlideCalTag, offset);


    }

    public double pow10floor(double x) {
        return Math.pow(10, Math.floor(Math.log(x) / Math.log(10)));
    }

    @Override
    protected Function getJoystickScalingFunction() {
        return Functions.none();
    }

    public enum Opts {
        TEAM_COLOR(teamColorTag), DO_SKYSTONE(doSkyStoneTag), VERTICAL_SLIDE_CALIBRATION(verticalSlideCalTag);


        //        public boolean b;
//        public double f;
        public String s;

        Opts(String s) {
            this.s = s;
        }
    }

}
