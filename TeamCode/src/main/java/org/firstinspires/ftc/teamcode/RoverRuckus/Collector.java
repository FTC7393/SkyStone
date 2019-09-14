package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.qualcomm.robotcore.hardware.ColorSensor;

import evlib.hardware.motors.Motor;
import evlib.hardware.servos.ServoControl;

/**
 * Created by ftc7393 on 10/27/2018.
 */

public class Collector {

    private Motor collection;
    private ServoControl doorRight;
    private ServoControl doorLeft;
    private ColorSensor left;
    private ColorSensor right;
//    public Collector(Motor collection, ServoControl door, ColorSensor left, ColorSensor right) {

    public Collector(Motor collection, ServoControl doorRight, ServoControl doorLeft) {
        this.collection = collection;
        this.doorRight = doorRight;
        this.doorLeft = doorLeft;
//        this.left = left;
//        this.right = right;

    }

    public void act() {

        collection.update();
       ;
    }
    public void collectionPower(double a){collection.setPower(a);}
    public void openRightDoor(){doorRight.goToPreset(RoverRuckusRobotCfg.DoorRightPresets.OPEN);}
    public void closeRightDoor(){doorRight.goToPreset(RoverRuckusRobotCfg.DoorRightPresets.CLOSE);}
    public void openLeftDoor(){doorLeft.goToPreset(RoverRuckusRobotCfg.DoorLeftPresets.OPEN);}
    public void closeLeftDoor(){doorLeft.goToPreset(RoverRuckusRobotCfg.DoorLeftPresets.CLOSE);}
}
