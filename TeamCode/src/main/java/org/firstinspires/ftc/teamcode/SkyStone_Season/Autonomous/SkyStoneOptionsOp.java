package org.firstinspires.ftc.teamcode.SkyStone_Season.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import ftc.electronvolts.util.Function;
import ftc.electronvolts.util.Functions;
import ftc.electronvolts.util.TeamColor;
import ftc.evlib.opmodes.AbstractOptionsOp;


/**
 * Created by ftc7393 on 12/6/2017.
 */
@TeleOp(name = "OptionsOp")
public class SkyStoneOptionsOp extends AbstractOptionsOp {

    public static final String FILENAME = "options_skystone.txt";
    public static final String teamColorTag = "teamColor";
    public static final TeamColor teamColorDefault = TeamColor.BLUE;
    int index = 0;
    private Opts[] values;

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

        if (driver1.right_bumper.justPressed()) {
            if (values[index] == Opts.TEAM_COLOR) {
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
        if (values[index] == Opts.TEAM_COLOR) {
            telemetry.addData("teamColor", optionsFile.get(values[index].s, teamColorDefault));
            telemetry.addData("index", index);
        }


    }

    public double pow10floor(double x) {
        return Math.pow(10, Math.floor(Math.log(x) / Math.log(10)));
    }

    @Override
    protected Function getJoystickScalingFunction() {
        return Functions.none();
    }

    public enum Opts {
        TEAM_COLOR("teamcolor");

        //        public boolean b;
//        public double f;
        public String s;

        Opts(String s) {
            this.s = s;
        }
    }

}
