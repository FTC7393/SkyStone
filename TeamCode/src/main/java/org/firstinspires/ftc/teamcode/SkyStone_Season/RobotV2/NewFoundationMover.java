package org.firstinspires.ftc.teamcode.SkyStone_Season.RobotV2;

import org.firstinspires.ftc.teamcode.SkyStone_Season.SkystoneRobotCfg;

import ftc.evlib.hardware.servos.ServoControl;

public class NewFoundationMover {

    private ServoControl rightFoundationMoverServo, leftFoundationMoverServo;

    public NewFoundationMover(ServoControl rightServo, ServoControl leftServo) {
        this.rightFoundationMoverServo = rightServo;
        this.leftFoundationMoverServo = leftServo;
    }

    public void servosDown() {
        leftFoundationMoverServo.goToPreset(SkystoneRobotCfgV2.LeftFoundationMoverServoPresets.DOWN);
        rightFoundationMoverServo.goToPreset(SkystoneRobotCfgV2.RightFoundationMoverServoPresets.DOWN);
    }

    public void servosUp() {
        leftFoundationMoverServo.goToPreset(SkystoneRobotCfgV2.LeftFoundationMoverServoPresets.UP);
        rightFoundationMoverServo.goToPreset(SkystoneRobotCfgV2.RightFoundationMoverServoPresets.UP);
    }

    public void servosReady() {
        leftFoundationMoverServo.goToPreset(SkystoneRobotCfgV2.LeftFoundationMoverServoPresets.READY);
        rightFoundationMoverServo.goToPreset(SkystoneRobotCfgV2.RightFoundationMoverServoPresets.READY);
    }
}
