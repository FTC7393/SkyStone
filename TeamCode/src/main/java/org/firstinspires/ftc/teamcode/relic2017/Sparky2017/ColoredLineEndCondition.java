package org.firstinspires.ftc.teamcode.relic2017.Sparky2017;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;

import ftc.electronvolts.statemachine.EndCondition;
import ftc.electronvolts.util.InputExtractor;

/**
 * Created by ftc7393 on 2/4/2018.
 */

public class ColoredLineEndCondition implements EndCondition, InputExtractor {

    private final ColorSensor colorSensor;

    public ColoredLineEndCondition(ColorSensor colorSensor) {
        this.colorSensor = colorSensor;
    }

    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F, 0F, 0F};

    // values is a reference to the hsvValues array.

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    final double SCALE_FACTOR = 255;
    boolean isDone = false;

    @Override
    public void init() {



    }


    @Override
    public boolean isDone() {

        Color.RGBToHSV((int) (colorSensor.red() * SCALE_FACTOR),
                (int) (colorSensor.green() * SCALE_FACTOR),
                (int) (colorSensor.blue() * SCALE_FACTOR),
                hsvValues);

        if (hsvValues[0] > 100) {
            isDone = true;
        }

        return isDone;
    }



    @Override
    public Object getValue() {
        return hsvValues[0];
    }
}
