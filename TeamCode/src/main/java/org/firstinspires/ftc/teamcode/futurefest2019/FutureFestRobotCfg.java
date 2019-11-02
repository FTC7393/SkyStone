package org.firstinspires.ftc.teamcode.futurefest2019;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Map;

import ftc.electronvolts.util.units.Distance;
import ftc.electronvolts.util.units.Time;
import ftc.electronvolts.util.units.Velocity;
import ftc.evlib.hardware.config.RobotCfg;
import ftc.evlib.hardware.control.MecanumControl;
import ftc.evlib.hardware.motors.MecanumMotors;
import ftc.evlib.hardware.motors.MotorEnc;
import ftc.evlib.hardware.motors.Motors;
import ftc.evlib.hardware.servos.ServoCfg;
import ftc.evlib.hardware.servos.ServoControl;
import ftc.evlib.hardware.servos.ServoName;
import ftc.evlib.hardware.servos.Servos;

/**
 * Created by ftc7393 on 9/22/2018.
 */

public class FutureFestRobotCfg extends RobotCfg {

//    public enum DumpServoPresets {
//        CLOSED,
//        OPEN
//    }
//
//    /**
//     * defines all the servos on the robot
//     */
//    public enum FutureFestServoEnum implements ServoName {
//        //enum name("hardware name", preset enum.values()),
//        DUMP_SERVO("dumpServo", DumpServoPresets.values());
//
//        private final String hardwareName;
//        private final Enum[] presets;
//
//        FutureFestServoEnum(String hardwareName, Enum[] presets) {
//            this.hardwareName = hardwareName;
//            this.presets = presets;
//        }
//
//        @Override
//        public String getHardwareName() {
//            return hardwareName;
//        }
//
//        @Override
//        public Enum[] getPresets() {
//            return presets;
//        }
//
//        @Override
//        public Class<FutureFestRobotCfg> getRobotCfg() {
//            return FutureFestRobotCfg.class;
//        }
//    }




    private final MecanumControl mecanumControl;
    private Servos servos;
    private DcMotor collector = null;
    private final DcMotor dumpMotor;

    private static final Velocity MAX_ROBOT_SPEED = new Velocity(Distance.fromInches(57 * 4), Time.fromSeconds(2.83));
    private static final Velocity MAX_ROBOT_SPEED_SIDEWAYS = new Velocity(Distance.fromInches(21.2441207039), Time.fromSeconds(1));

    public FutureFestRobotCfg(HardwareMap hardwareMap) {
        this(hardwareMap, ServoCfg.defaultServoStartPresetMap(new ServoName[0]));
    }
    public FutureFestRobotCfg(HardwareMap hardwareMap, Map<ServoName, Enum> servoStartPresetMap) {
        super(hardwareMap);
        double scaleFactor = 1.0;
        mecanumControl = new MecanumControl(new MecanumMotors(
                Motors.withEncoder(hardwareMap.dcMotor.get("fl"), false, true, stoppers), // 0
                Motors.withEncoder(hardwareMap.dcMotor.get("fr") , true, true, stoppers), // 1
                Motors.scale(Motors.withEncoder(hardwareMap.dcMotor.get("br") , false, true, stoppers),scaleFactor), // 2
                Motors.scale(Motors.withEncoder(hardwareMap.dcMotor.get("bl") , true, true, stoppers),scaleFactor), // 3
                true, MAX_ROBOT_SPEED,MAX_ROBOT_SPEED_SIDEWAYS));
        collector  = hardwareMap.get(DcMotor.class, "collection");
        collector.setPower(0);
        collector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dumpMotor = hardwareMap.get(DcMotor.class,"dumpMotor");

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

    public DcMotor getDumpMotor() {
        return dumpMotor;
    }
}
