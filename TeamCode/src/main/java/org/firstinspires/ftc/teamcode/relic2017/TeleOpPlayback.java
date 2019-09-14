package org.firstinspires.ftc.teamcode.relic2017;

import com.google.common.io.BaseEncoding;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.exception.RobotCoreException;

import org.firstinspires.ftc.teamcode.relic2017.Sparky2017.TeleOp2017;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

import ftc.evlib.util.EVConverters;
import ftc.evlib.util.FileUtil;
import ftc.electronvolts.util.files.Logger;
import ftc.electronvolts.util.files.OptionsFile;

/**
 * This file was made by the electronVolts, FTC team 7393
 * Date Created: 10/11/16
 */
@Autonomous(name = "TeleOpPlayback")
@Disabled
public class TeleOpPlayback extends TeleOp2017 {
    public static final String GAMEPAD_1_TITLE = "gamepad1";
    public static final String GAMEPAD_2_TITLE = "gamepad2";

    private BufferedReader bufferedReader = null;
    private boolean endOfFile = false;
    private int gamepad1Index, gamepad2Index;

    @Override
    protected void setup() {
        super.setup();

        OptionsFile optionsFile = new OptionsFile(EVConverters.getInstance(), FileUtil.getOptionsFile(TeleOpPlaybackOptions.FILENAME));
        File file = new File(optionsFile.get(TeleOpPlaybackOptions.filepathTag, String.class));

        endOfFile = true;
        try {
            bufferedReader = new BufferedReader(new FileReader(file));
            String line = bufferedReader.readLine();
            if (line != null) {
                endOfFile = false;
                String[] titles = line.split("\t");
                for (int i = 0; i < titles.length; i++) {
                    if (titles[i].equals(GAMEPAD_1_TITLE)) {
                        gamepad1Index = i;
                    } else if (titles[i].equals(GAMEPAD_2_TITLE)) {
                        gamepad2Index = i;
                    }
                }
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Override
    protected Logger createLogger() {
        return new Logger("teleop", ".csv", robotCfg.getLoggerColumns());
    }

    @Override
    protected void act() {
        if (endOfFile) {
            telemetry.addData("End of file", 0);
        } else {
            try {
                String line = bufferedReader.readLine();
                if (line == null) {
                    endOfFile = true;
                } else {
                    String[] values = line.split("\t");
                    gamepad1.fromByteArray(BaseEncoding.base64Url().decode(values[gamepad1Index]));
                    gamepad2.fromByteArray(BaseEncoding.base64Url().decode(values[gamepad2Index]));
                }
            } catch (IOException | RobotCoreException e) {
                endOfFile = true;
                stop();
                try {
                    bufferedReader.close();
                } catch (IOException e1) {
                    e1.printStackTrace();
                }
                return;
            }
            super.act();
        }
    }
}
