package org.firstinspires.ftc.teamcode.SkyStone_Season.TeleOp;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.SkyStone_Season.SkystoneRobotCfg;

import ftc.evlib.hardware.servos.ServoControl;

public class FoundationMover {

    private ServoControl rightfoundationmoverServo, leftfoundationmoverServo;

    public FoundationMover(ServoControl rightServo, ServoControl leftServo) {
        this.rightfoundationmoverServo = rightServo;
        this.leftfoundationmoverServo = leftServo;
    }

    public void servosDown() {
        leftfoundationmoverServo.goToPreset(SkystoneRobotCfg.LeftFoundationMoverServoPresets.DOWN);
        rightfoundationmoverServo.goToPreset(SkystoneRobotCfg.RightFoundationMoverServoPresets.DOWN);
    }
    public void servosUp() {
        leftfoundationmoverServo.goToPreset(SkystoneRobotCfg.LeftFoundationMoverServoPresets.UP);
        rightfoundationmoverServo.goToPreset(SkystoneRobotCfg.RightFoundationMoverServoPresets.UP);
    }
}
