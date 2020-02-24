package org.firstinspires.ftc.teamcode.SkyStone_Season.RobotV2;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import ftc.electronvolts.statemachine.State;
import ftc.evlib.hardware.motors.Motor;

public class BlockCollector {

    private Motor collectorMotor;
    private Rev2mDistanceSensor blockDetector;
    private State currentState;

    public Motor getCollectorMotor() {
        return collectorMotor;
    }

    public BlockCollector(Motor collectorMotor, Rev2mDistanceSensor blockDetector){
        this.collectorMotor = collectorMotor;
        this.blockDetector = blockDetector;
    }

    public void act() {
        collectorMotor.update();
    }

    public void setPower(double newPower) {
        collectorMotor.setPower(newPower);
    }

    public void stop() {
        collectorMotor.setPower(0);
    }
}

