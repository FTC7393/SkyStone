package org.firstinspires.ftc.teamcode.SkyStone_Season.RobotV2;

import ftc.evlib.hardware.motors.Motor;

public class BlockCollector {

    private Motor collectorMotor;

    public BlockCollector(Motor collectorMotor){
        this.collectorMotor = collectorMotor;
    }

    public void act() {
        collectorMotor.update();
    }

    public void setPower (double newPower) {
        collectorMotor.setPower(newPower);
    }

    public void stop() {
        collectorMotor.setPower(0);
    }
}



