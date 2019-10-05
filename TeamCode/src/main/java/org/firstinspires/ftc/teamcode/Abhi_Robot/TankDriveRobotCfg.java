package org.firstinspires.ftc.teamcode.Abhi_Robot;

/**
 * Created by ftc7393 on 7/31/2018.
 */

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Map;

import ftc.electronvolts.util.units.Distance;
import ftc.electronvolts.util.units.Time;
import ftc.electronvolts.util.units.Velocity;
import ftc.evlib.hardware.config.RobotCfg;
import ftc.evlib.hardware.motors.Motors;
import ftc.evlib.hardware.motors.TwoMotors;
import ftc.evlib.hardware.servos.ServoCfg;
import ftc.evlib.hardware.servos.ServoControl;
import ftc.evlib.hardware.servos.ServoName;
import ftc.evlib.hardware.servos.Servos;

/**
 * This file was made by the electronVolts, FTC team 7393
 * Date Created: 9/12/16
 * <p>
 * The HardwareCfg for the simple fair parade robot with 2 motors.
 */
public class TankDriveRobotCfg extends RobotCfg {
    /**
     * The speed the robot moves at when the power is set to 100%
     */
    private static final Velocity maxRobotSpeed = new Velocity(Distance.fromFeet(1), Time.fromSeconds(1));

    private final TwoMotors twoMotors;
    private final ServoControl clamp;
    private final Servos servos;


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
        public Class<TankDriveRobotCfg> getRobotCfg() {
            return TankDriveRobotCfg.class;
        }
    }

    public TankDriveRobotCfg(HardwareMap hardwareMap) {
        this(hardwareMap, ServoCfg.defaultServoStartPresetMap(TankDriveServoEnum.values()));
    }


    public TankDriveRobotCfg(HardwareMap hardwareMap, Map<ServoName, Enum> servoStartPresetMap) {
        super(hardwareMap);

        //get the two motors from the hardwareMap and put them into a TwoMotors object
        twoMotors = new TwoMotors(
                Motors.withoutEncoder(hardwareMap.dcMotor.get("leftmotor"), false, false, stoppers),
                Motors.withoutEncoder(hardwareMap.dcMotor.get("rightmotor"), true, false, stoppers),
                false, maxRobotSpeed
        );

        servos = new Servos(ServoCfg.createServoMap(hardwareMap, servoStartPresetMap));

        clamp = getServo(TankDriveServoEnum.CLAMP_SERVO);

    }

    @Override
    public void start() {


    }

    @Override
    public void act() {
        twoMotors.update();
    }

    @Override
    public void stop() {
        twoMotors.stop();
    }

    public TwoMotors getTwoMotors() {
        return twoMotors;
    }

    public ServoControl getClamp(TankDriveServoEnum clampServo) {return clamp;}

}

