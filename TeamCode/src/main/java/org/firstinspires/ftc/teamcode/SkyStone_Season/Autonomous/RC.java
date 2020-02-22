package org.firstinspires.ftc.teamcode.SkyStone_Season.Autonomous;

import ftc.electronvolts.util.units.Angle;
import ftc.evlib.hardware.control.RotationControl;
import ftc.evlib.hardware.control.RotationControls;
import ftc.evlib.hardware.sensors.Gyro;

public class RC {

    private final double maxAngularSpeed;
    private final Angle tolerance;
    private final double gyroGain;

    public RC(double maxAngularSpeed, Angle tolerance, double gyroGain, Gyro gyro) {
        this.maxAngularSpeed = maxAngularSpeed;
        this.tolerance = tolerance;
        this.gyroGain = gyroGain;
        this.gyro = gyro;
    }

    private final Gyro gyro;

    public RotationControl gyro(double angleInDegrees) {
        return RotationControls.gyro(gyro, gyroGain, Angle.fromDegrees(angleInDegrees), tolerance, maxAngularSpeed);
    }

}
