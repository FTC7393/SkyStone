package org.firstinspires.ftc.teamcode.SkyStone_Season.TeleOp;

import org.firstinspires.ftc.teamcode.SkyStone_Season.SkystoneRobotCfg;

import ftc.evlib.hardware.servos.ServoControl;

public class FoundationMover {

    private ServoControl rightFoundationMoverServo, leftFoundationMoverServo;

    public FoundationMover(ServoControl rightServo, ServoControl leftServo) {
        this.rightFoundationMoverServo = rightServo;
        this.leftFoundationMoverServo = leftServo;
    }

    public void servosDown() {
        leftFoundationMoverServo.goToPreset(SkystoneRobotCfg.LeftFoundationMoverServoPresets.DOWN);
        rightFoundationMoverServo.goToPreset(SkystoneRobotCfg.RightFoundationMoverServoPresets.DOWN);
    }
    public void servosUp() {
        leftFoundationMoverServo.goToPreset(SkystoneRobotCfg.LeftFoundationMoverServoPresets.UP);
        rightFoundationMoverServo.goToPreset(SkystoneRobotCfg.RightFoundationMoverServoPresets.UP);
    }
}
