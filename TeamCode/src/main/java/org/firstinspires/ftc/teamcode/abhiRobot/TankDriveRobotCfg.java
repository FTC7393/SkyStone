package org.firstinspires.ftc.teamcode.abhiRobot;

/**
 * Created by ftc7393 on 7/31/2018.
 */

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Map;

import ftc.electronvolts.util.units.Distance;
import ftc.electronvolts.util.units.Time;
import ftc.electronvolts.util.units.Velocity;
import ftc.evlib.hardware.config.RobotCfg;
import ftc.evlib.hardware.motors.Motors;
import ftc.evlib.hardware.motors.TwoMotors;
import ftc.evlib.hardware.servos.ServoControl;
import ftc.evlib.hardware.servos.ServoName;

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

    public enum clampServoPresets {
        OPEN,
        CLOSE
    }



    public enum TankDriveRobotServoEnum implements ServoName {
        //enum name("hardware name", preset enum.values()),
        CLAMP_SERVO("clampServo", TankDriveRobotCfg.clampServoPresets.values());

        private final String hardwareName;
        private final Enum[] presets;

        TankDriveRobotServoEnum(String hardwareName, Enum[] presets) {
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

    private final TwoMotors twoMotors;
    private final Servo clamp;


    public TankDriveRobotCfg(HardwareMap hardwareMap) {
        super(hardwareMap);

        //get the two motors from the hardwareMap and put them into a TwoMotors object
        twoMotors = new TwoMotors(
                Motors.withoutEncoder(hardwareMap.dcMotor.get("leftmotor"), false, false, stoppers),
                Motors.withoutEncoder(hardwareMap.dcMotor.get("rightmotor"), true, false, stoppers),
                false, maxRobotSpeed
        );



    }

    @Override
    public void start() {}

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
    public ServoControl getClamp(TankDriveRobotServoEnum clampServo) {return clamp;}
}