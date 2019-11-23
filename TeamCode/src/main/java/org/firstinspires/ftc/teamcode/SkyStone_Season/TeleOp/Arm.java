package org.firstinspires.ftc.teamcode.SkyStone_Season.TeleOp;


import com.qualcomm.robotcore.hardware.Servo;
import ftc.evlib.hardware.servos.ServoControl;

public class LiftArmClass {

    private Servo elbow, wrist, fingers;

    public LiftArmClass(Servo elbow, Servo wrist, Servo fingers){
        this.elbow = elbow;
        this.wrist = wrist;
        this.fingers = fingers;
    }

    public void Extend() {
        elbow.goToPreset(.EXTEND);
        wrist.goToPreset(.EXTEND);

    }

    public void Retract() {
        elbow.goToPreset(.RETRACT);
        wrist.goToPreset(.RETRACT);
    }

    public void Grab() {
        fingers.goToPreset(.GRAB);
    }

    public void Release() {
        fingers.goToPreset(.RELEASE)
    }

}
