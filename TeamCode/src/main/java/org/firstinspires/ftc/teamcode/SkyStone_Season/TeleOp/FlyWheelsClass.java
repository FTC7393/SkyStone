package org.firstinspires.ftc.teamcode.SkyStone_Season.TeleOp;

import org.firstinspires.ftc.teamcode.SkyStone_Season.SkystoneRobotCfg;

import ftc.evlib.hardware.motors.Motor;

public class FlyWheelsClass {

    private Motor leftFlywheel,rightFlywheel;

    public FlyWheelsClass(Motor leftFlywheel, Motor rightFlywheel){
        this.leftFlywheel = leftFlywheel;
        this.rightFlywheel = rightFlywheel;
    }

    public void act() {
        leftFlywheel.update();
        rightFlywheel.update();
    }

    public void setPower (double newPower) {
        leftFlywheel.setPower(newPower);
        rightFlywheel.setPower(-newPower);
    }

    public void stop() {
        leftFlywheel.setPower(0);
        rightFlywheel.setPower(0);
    }

}
