package org.firstinspires.ftc.teamcode.relic2017;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by ftc7393 on 12/29/2017.
 */

public class TestColor {
    ColorSensor left_color_sensor;
    ColorSensor right_color_sensor;
    float hsvValues[] = {0F, 0F, 0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    final double SCALE_FACTOR = 255;
//    boolean isDone=false;
    public double getHue(){
        return hsvValues[0];
    }







}
