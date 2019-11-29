package org.firstinspires.ftc.teamcode.SkyStone_Season.TeleOp;

import ftc.evlib.hardware.servos.ServoControl;

public class foundationMover {

    private ServoControl rightServo, leftServo;

    public enum rightServoPositions {
        UP,
        DOWN
    }
    public enum leftServoPositions {
        UP,
        DOWN
    }


    public foundationMover ( ServoControl rightServo, ServoControl leftServo) {
        this.rightServo = rightServo;
        this.leftServo = leftServo;
    }

    public void servosDown() {

    }


    public void act() {
        rightServo.act();
        leftServo.act();
    }
}
