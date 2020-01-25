package org.firstinspires.ftc.teamcode.SkyStone_Season.RobotV2;

import ftc.evlib.hardware.motors.Motor;

public class BlockCollector {

    private Motor CollectorMotor;

    public BlockCollector(Motor CollectorMotor){
        this.CollectorMotor = CollectorMotor;
    }

    public void act() {
        CollectorMotor.update();
    }

    public void setPower (double newPower) {
        CollectorMotor.setPower(newPower);
    }

    public void stop() {
        CollectorMotor.setPower(0);
    }
}

