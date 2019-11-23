package org.firstinspires.ftc.teamcode.SkyStone_Season.TeleOp;


import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.SkyStone_Season.SkystoneRobotCfg;

import ftc.evlib.hardware.servos.ServoControl;

public class LiftArm {

    private ServoControl elbow, wrist, fingers;

    public LiftArm(ServoControl elbow, ServoControl wrist, ServoControl fingers){
        this.elbow = elbow;
        this.wrist = wrist;
        this.fingers = fingers;
    }

    public void Extend() {
        elbow.goToPreset(SkystoneRobotCfg.ElbowServoPresets.EXTEND);
        wrist.goToPreset(SkystoneRobotCfg.WristServoPresets.EXTEND);

    }

    public void Retract() {
        elbow.goToPreset(SkystoneRobotCfg.ElbowServoPresets.RETRACT);
        wrist.goToPreset(SkystoneRobotCfg.WristServoPresets.RETRACT);
    }

    public void Grab() {
        fingers.goToPreset(SkystoneRobotCfg.FingersServoPresets.GRAB);
    }

    public void Release() {
        fingers.goToPreset(SkystoneRobotCfg.FingersServoPresets.RELEASE);
    }

}
