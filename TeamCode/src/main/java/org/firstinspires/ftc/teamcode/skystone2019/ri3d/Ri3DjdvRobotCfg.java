package org.firstinspires.ftc.teamcode.skystone2019.ri3d;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Map;

import ftc.electronvolts.util.units.Distance;
import ftc.electronvolts.util.units.Time;
import ftc.electronvolts.util.units.Velocity;
import ftc.evlib.hardware.config.RobotCfg;
import ftc.evlib.hardware.control.MecanumControl;
import ftc.evlib.hardware.motors.MecanumMotors;
import ftc.evlib.hardware.motors.Motors;
import ftc.evlib.hardware.servos.ServoCfg;
import ftc.evlib.hardware.servos.ServoName;
import ftc.evlib.hardware.servos.Servos;

/**
 * Created by ftc7393 on 9/22/2018.
 */

public class Ri3DjdvRobotCfg extends RobotCfg {

    public enum GrabServoPresets {
        OPEN, CLOSED
    }
    public enum SpinServoPresets {
        LEFT, CENTER, RIGHT;
    }
    public enum RotateServoPresets {
        RETRACT,DROP,GRAB;
    }

    /**
     * defines all the servos on the robot
     */
    public enum Ri3DServoEnum implements ServoName {
        //enum name("hardware name", preset enum.values()),
        ROTATE_SERVO("rotateServo", RotateServoPresets.values()),
        SPIN_SERVO("spinServo", SpinServoPresets.values()),
        GRAB_SERVO("grabServo", GrabServoPresets.values());

        private final String hardwareName;
        private final Enum[] presets;

        Ri3DServoEnum(String hardwareName, Enum[] presets) {
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
        public Class<Ri3DjdvRobotCfg> getRobotCfg() {
            return Ri3DjdvRobotCfg.class;
        }
    }




    private final MecanumControl mecanumControl;
    private Servos servos;
    private DcMotor lift = null;

    private static final Velocity MAX_ROBOT_SPEED = new Velocity(Distance.fromInches(4*57 * 3), Time.fromSeconds(2.83));
    private static final Velocity MAX_ROBOT_SPEED_SIDEWAYS = new Velocity(Distance.fromInches(21.2441207039), Time.fromSeconds(1));

    public Ri3DjdvRobotCfg(HardwareMap hardwareMap) {
        this(hardwareMap, ServoCfg.defaultServoStartPresetMap(Ri3DServoEnum.values()));
    }
    public Ri3DjdvRobotCfg(HardwareMap hardwareMap, Map<ServoName, Enum> servoStartPresetMap) {
        super(hardwareMap);
        double scaleFactor = 1.0;
        mecanumControl = new MecanumControl(new MecanumMotors(
                Motors.withEncoder(hardwareMap.dcMotor.get("frontLeft"), true, true, stoppers), // 0
                Motors.withEncoder(hardwareMap.dcMotor.get("frontRight") , false, true, stoppers), // 1
                Motors.scale(Motors.withEncoder(hardwareMap.dcMotor.get("backRight") , false, true, stoppers),scaleFactor), // 2
                Motors.scale(Motors.withEncoder(hardwareMap.dcMotor.get("backLeft") , true, true, stoppers),scaleFactor), // 3
                true, MAX_ROBOT_SPEED,MAX_ROBOT_SPEED_SIDEWAYS));
        lift = hardwareMap.get(DcMotor.class, "lift");
        lift.setPower(0);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        servos = new Servos(ServoCfg.createServoMap(hardwareMap, servoStartPresetMap));
    }
    @Override
    public void start() {

    }

    @Override
    public void act() {
        mecanumControl.act();
        servos.act();
    }

    @Override
    public void stop() {
        mecanumControl.stop();


    }
    public Servos getServos(){return servos;}
    public DcMotor getLift(){return lift;}
    public MecanumControl getMecanumControl() {
        return mecanumControl;
    }

}
