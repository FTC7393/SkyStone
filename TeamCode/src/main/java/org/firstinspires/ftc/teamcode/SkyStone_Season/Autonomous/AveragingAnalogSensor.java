package org.firstinspires.ftc.teamcode.SkyStone_Season.Autonomous;

import ftc.evlib.hardware.sensors.AnalogSensor;

public class AveragingAnalogSensor implements AnalogSensor {
    private final AnalogSensor sensor;
    private double lastValue = 50; //this is terrible but better than 255

    public AveragingAnalogSensor(AnalogSensor sensor) {
        this.sensor = sensor;
    }

    @Override
    public Double getValue() {
        double x = sensor.getValue();
        if(x >= 255) {
            return lastValue;
        }
        lastValue = x;
        return x;
    }
}
