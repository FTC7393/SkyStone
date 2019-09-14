package org.firstinspires.ftc.teamcode.RoverRuckus.Sandbox;

import com.google.common.collect.ImmutableList;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

import ftc.evlib.hardware.config.RobotCfg;
import ftc.evlib.hardware.control.MecanumControl;
import ftc.evlib.hardware.motors.MecanumMotors;
import ftc.evlib.hardware.motors.MotorEnc;
import ftc.evlib.hardware.motors.Motors;
import ftc.evlib.hardware.sensors.Gyro;
import ftc.evlib.hardware.sensors.IMUGyro;
import ftc.evlib.statemachine.EVEndConditions;
import ftc.electronvolts.util.InputExtractor;
import ftc.electronvolts.util.files.Logger;
import ftc.electronvolts.util.units.Distance;
import ftc.electronvolts.util.units.Time;
import ftc.electronvolts.util.units.Velocity;

public class RoverCfg extends RobotCfg {

    private static final Velocity MAX_ROBOT_SPEED = new Velocity(Distance.fromInches(57 * 4), Time.fromSeconds(2.83));
    private static final Velocity MAX_ROBOT_SPEED_SIDEWAYS = new Velocity(Distance.fromInches(21.2441207039), Time.fromSeconds(1));

    private final MecanumControl mecanumControl;
    private final IMUGyro gyro;
    private final ImmutableList<Logger.Column> loggerColumns;

    public RoverCfg(HardwareMap hmap) {
        super(hmap);

        //leftColorSensor = hardwareMap.colorSensor.get(LEFT_COLOR_SENSOR_NAME);
        //rightColorSensor = hardwareMap.colorSensor.get(RIGHT_COLOR_SENSOR_NAME);
        MotorEnc frontLeft = Motors.withEncoder(hmap.dcMotor.get("frontLeft"), true, true, stoppers);
        MotorEnc frontRight = Motors.withEncoder(hmap.dcMotor.get("frontRight"), false, true, stoppers);
        MotorEnc backLeft = Motors.withEncoder(hmap.dcMotor.get("backLeft"), true, true, stoppers);
        MotorEnc backRight = Motors.withEncoder(hmap.dcMotor.get("backRight"), false, true, stoppers);

        boolean useSpeedMode = true;
        Velocity maxRobotSpeed = new Velocity(Distance.fromFeet(8), Time.fromSeconds(1));
        MecanumMotors mecMotors = new MecanumMotors(frontLeft, frontRight, backLeft,
                backRight, useSpeedMode, MAX_ROBOT_SPEED, MAX_ROBOT_SPEED_SIDEWAYS);
        mecanumControl = new MecanumControl(mecMotors);

        gyro = new IMUGyro(hmap.get(BNO055IMU.class, "imu"));

        loggerColumns = ImmutableList.of(
                new Logger.Column("gyro", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return gyro.getHeading();
                    }
                }),
                new Logger.Column("velocityR", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return mecanumControl.getVelocityR();
                    }
                }),
                new Logger.Column("Motor 0", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return mecanumControl.getMecanumMotors().getValue(0);
                    }
                }),
                new Logger.Column("Tolerance", new InputExtractor<Double>() {
            @Override
            public Double getValue() {
                return EVEndConditions.toleranceLog ;
            }
        }  ),
                new Logger.Column("Separation", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return EVEndConditions.separationLog ;
                    }
                }

        ));

    }

    @Override
    public void start() {

    }

    @Override
    public void act() {
        mecanumControl.act();
    }

    @Override
    public void stop() {
        gyro.stop();
        mecanumControl.stop();
    }

    public Gyro getGyro() {
        return gyro;
    }

    public MecanumControl getMecanumControl() {
        return mecanumControl;
    }

    public List<Logger.Column> getLoggerColumns() {
        return loggerColumns;
    }
}
