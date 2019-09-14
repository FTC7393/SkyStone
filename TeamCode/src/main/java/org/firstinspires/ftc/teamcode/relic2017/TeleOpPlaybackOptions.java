package org.firstinspires.ftc.teamcode.relic2017;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;
import java.io.FilenameFilter;
import java.util.Arrays;

import ftc.evlib.opmodes.AbstractOptionsOp;
import ftc.evlib.util.FileUtil;
import ftc.electronvolts.util.Function;
import ftc.electronvolts.util.Functions;

/**
 * This file was made by the electronVolts, FTC team 7393
 * Date Created: 12/14/16
 */
@TeleOp(name = "TeleOpPlaybackOptions")
@Disabled
public class TeleOpPlaybackOptions extends AbstractOptionsOp {
    public static final String FILENAME = "PlaybackOptions.txt";
    public static final String filepathTag = "filepath";

    private File[] files;
    private int fileIndex = 0;
    private boolean indexChanged = true;

    public TeleOpPlaybackOptions() {
        super(FILENAME);
    }

    @Override
    protected Function getJoystickScalingFunction() {
        return Functions.none();
    }

    @Override
    protected void setup() {
        super.setup();
        //look through the logs dir and find the teleop[...].csv files
        files = FileUtil.getLogsDir().listFiles(new FilenameFilter() {
            public boolean accept(File dir, String name) {
                return name.endsWith("teleop.csv");
            }
        });
        Arrays.sort(files);
    }

    @Override
    protected void act() {
        if (driver1.dpad_up.justPressed()) {
            fileIndex++;
            if (fileIndex > files.length - 1) fileIndex = 0;
            indexChanged = true;
        }
        if (driver1.dpad_down.justPressed()) {
            fileIndex--;
            if (fileIndex < 0) fileIndex = files.length - 1;
            indexChanged = true;
        }

        if (indexChanged) {
            indexChanged = false;
            optionsFile.set(filepathTag, files[fileIndex].getAbsolutePath());
        }

        telemetry.addData("fileIndex", fileIndex);
        telemetry.addData("* dpad up/down => " + filepathTag, optionsFile.get(filepathTag, String.class, null));
    }
}