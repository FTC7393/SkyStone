package org.firstinspires.ftc.teamcode.Abhi_Robot.mechanumRobotControl;

/**
 * Created by ftc7393 on 7/31/2018.
 */

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Map;

import ftc.electronvolts.util.units.Distance;
import ftc.electronvolts.util.units.Time;
import ftc.electronvolts.util.units.Velocity;
import ftc.evlib.hardware.config.RobotCfg;
import ftc.evlib.hardware.motors.MecanumMotors;
import ftc.evlib.hardware.motors.MotorEnc;
import ftc.evlib.hardware.motors.Motors;
import ftc.evlib.hardware.sensors.IMUGyro;
import ftc.evlib.hardware.servos.ServoCfg;
import ftc.evlib.hardware.servos.ServoControl;
import ftc.evlib.hardware.servos.ServoName;
import ftc.evlib.hardware.servos.Servos;
import ftc.evlib.hardware.control.MecanumControl;

/**
 * This file was made by the electronVolts, FTC team 7393
 * Date Created: 9/12/16
 * <p>
 * The HardwareCfg for the simple fair parade robot with 2 motors.
 */
public class MechanumRobotCfg extends RobotCfg {
    /**
     * The speed the robot moves at when the power is set to 100%
     */
    private static final Velocity MAX_ROBOT_SPEED = new Velocity(Distance.fromFeet(1), Time.fromSeconds(1));
    private static final Velocity MAX_ROBOT_SPEED_SIDEWAYS = new Velocity(Distance.fromInches(21.2441207039), Time.fromSeconds(1));


    private final Servos servos;
    private final IMUGyro gyro;
    private final MecanumControl mecanumControl;
    private final MotorEnc frontLeft;
    private final MotorEnc frontRight;
    private final MotorEnc backLeft;
    private final MotorEnc backRight;


    public enum ClampServoPresets {
        CLOSED,
        OPEN
    }

    public enum TankDriveServoEnum implements ServoName {
        //enum name("hardware name", preset enum.values()),
        CLAMP_SERVO("clamp", ClampServoPresets.values());

        private final String hardwareName;
        private final Enum[] presets;

        TankDriveServoEnum(String hardwareName, Enum[] presets) {
            this.hardwareName = hardwareName;
            this.presets = presets;
        }

        @Override
        public String getHardwareName() {
            return hardwareName;
        }

        @Override
        public Enum[] getPresets() {
            return presets;
        }

        @Override
        public Class<MechanumRobotCfg> getRobotCfg() {
            return MechanumRobotCfg.class;
        }
    }



    public MechanumRobotCfg(HardwareMap hardwareMap, Map<ServoName, Enum> servoStartPresetMap) {
        super(hardwareMap);
        servos = new Servos(ServoCfg.createServoMap(hardwareMap, servoStartPresetMap));

        frontLeft=    Motors.withEncoder(hardwareMap.dcMotor.get("frontLeft"), false, true, stoppers);
        frontRight=   Motors.withEncoder(hardwareMap.dcMotor.get("frontRight") , true, true, stoppers);
        backLeft=     Motors.withEncoder(hardwareMap.dcMotor.get("backLeft") , false, true, stoppers);
        backRight=    Motors.withEncoder(hardwareMap.dcMotor.get("backRight") , true, true, stoppers);
//        backRight=    Motors.scale(Motors.withEncoder(hardwareMap.dcMotor.get("backRight") , false, true, stoppers), 0.7);

        gyro = new IMUGyro(hardwareMap.get(BNO055IMU.class, "imu"));
        mecanumControl = new MecanumControl(new MecanumMotors(
                frontLeft,
                frontRight,
                backLeft,
                backRight,
                true, MAX_ROBOT_SPEED,MAX_ROBOT_SPEED_SIDEWAYS));


    }

    @Override
    public void start() {}

    @Override
    public void act() {

    }

    @Override
    public void stop() {

    }


    public ServoControl getClamp() {
        return getServo(TankDriveServoEnum.CLAMP_SERVO);
    }

    @Override
    public Servos getServos() {
        return servos;
    }
}
