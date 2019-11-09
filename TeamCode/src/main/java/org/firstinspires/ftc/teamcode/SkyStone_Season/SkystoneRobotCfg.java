package org.firstinspires.ftc.teamcode.SkyStone_Season;

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
//comment
/**
 * Created by ftc7393 on 9/22/2018.
 */

public class SkystoneRobotCfg extends RobotCfg {

    public enum GrabServoPresets {
        CLOSE,
        OPEN
    }

    public enum RotateServoPresets {
        LEFT,
        MIDDLE,
        RIGHT
    }

    public enum extendServoPresets {
        RETRACT,
        EXTEND,
        GRAB
    }

    public enum pushServoPresets {
        RETRACT,
        EJECT
    }

    /**
     * defines all the servos on the robot
     */
    public enum FutureFestServoEnum implements ServoName {
        //enum name("hardware name", preset enum.values()),
        GRAB_SERVO("grabServo", GrabServoPresets.values()),
        ROTATE_SERVO("rotateServo", RotateServoPresets.values()),
        EXTEND_SERVO("extendServo", extendServoPresets.values());

        private final String hardwareName;
        private final Enum[] presets;

        FutureFestServoEnum(String hardwareName, Enum[] presets) {
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
        public Class<SkystoneRobotCfg> getRobotCfg() {
            return SkystoneRobotCfg.class;
        }
    }




    private final MecanumControl mecanumControl;
    private Servos servos;
    private DcMotor collector = null;

    private static final Velocity MAX_ROBOT_SPEED = new Velocity(Distance.fromInches(57 * 4), Time.fromSeconds(2.83));
    private static final Velocity MAX_ROBOT_SPEED_SIDEWAYS = new Velocity(Distance.fromInches(21.2441207039), Time.fromSeconds(1));

    public SkystoneRobotCfg(HardwareMap hardwareMap) {
        this(hardwareMap, ServoCfg.defaultServoStartPresetMap(FutureFestServoEnum.values()));
    }
    public SkystoneRobotCfg(HardwareMap hardwareMap, Map<ServoName, Enum> servoStartPresetMap) {
        super(hardwareMap);
        double scaleFactor = 1.0;
        mecanumControl = new MecanumControl(new MecanumMotors(
                Motors.withEncoder(hardwareMap.dcMotor.get("frontLeft"), true, true, stoppers), // 0
                Motors.withEncoder(hardwareMap.dcMotor.get("frontRight") , false, true, stoppers), // 1
                Motors.scale(Motors.withEncoder(hardwareMap.dcMotor.get("backRight") , true, true, stoppers),scaleFactor), // 2
                Motors.scale(Motors.withEncoder(hardwareMap.dcMotor.get("backLeft") , false, true, stoppers),scaleFactor), // 3
                true, MAX_ROBOT_SPEED,MAX_ROBOT_SPEED_SIDEWAYS));
        collector  = hardwareMap.get(DcMotor.class, "collection");
        collector.setPower(0);
        collector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
    public DcMotor getCollector(){return collector;}
    public MecanumControl getMecanumControl() {
        return mecanumControl;
    }

}
