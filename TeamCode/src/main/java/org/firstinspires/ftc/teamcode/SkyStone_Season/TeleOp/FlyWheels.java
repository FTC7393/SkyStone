package org.firstinspires.ftc.teamcode.SkyStone_Season.TeleOp;

import ftc.evlib.hardware.motors.Motor;

public class FlyWheels {

    private Motor collection;

    public FlyWheels(Motor Collection){
        this.collection = collection;
    }

    public void act() {
        collection.update();
    }

    public void collectionPower(double a){collection.setPower(a);}
}
