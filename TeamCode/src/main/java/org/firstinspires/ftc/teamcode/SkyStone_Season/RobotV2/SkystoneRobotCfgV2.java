package org.firstinspires.ftc.teamcode.SkyStone_Season.RobotV2;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;

import java.util.Map;

import ftc.electronvolts.util.Function;
import ftc.electronvolts.util.units.Distance;
import ftc.electronvolts.util.units.Time;
import ftc.electronvolts.util.units.Velocity;
import ftc.evlib.hardware.config.RobotCfg;
import ftc.evlib.hardware.control.MecanumControl;
import ftc.evlib.hardware.motors.MecanumMotors;
import ftc.evlib.hardware.motors.Motors;
import ftc.evlib.hardware.sensors.AveragedSensor;
import ftc.evlib.hardware.sensors.Gyro;
import ftc.evlib.hardware.sensors.IMUGyro;
import ftc.evlib.hardware.sensors.Sensors;
import ftc.evlib.hardware.servos.ServoCfg;
import ftc.evlib.hardware.servos.ServoControl;
import ftc.evlib.hardware.servos.ServoName;
import ftc.evlib.hardware.servos.Servos;
//comment

/**
 * Created by ftc7393 on 9/22/2018.
 */

public class SkystoneRobotCfgV2 extends RobotCfg {

    private final BlockCollector blockCollector;
    private final Gyro gyro0;
    private final LiftArmV2 liftArmV2;
    private final NewFoundationMover newFoundationMover;
    private final Rev2mDistanceSensor plusYDistanceSensor;
    private final Rev2mDistanceSensor minusYDistanceSensor;
//    private final AveragedSensor plusYDistanceSensor;



    public SkystoneRobotCfgV2(HardwareMap hardwareMap, Map<ServoName, Enum> servoStartPresetMap) {
        super(hardwareMap);
        double scaleFactor = 1.0;
        mecanumControl = new MecanumControl(new MecanumMotors(
                Motors.withEncoder(hardwareMap.dcMotor.get("backLeft"), true, true, stoppers), // 0
                Motors.withEncoder(hardwareMap.dcMotor.get("frontLeft"), false, true, stoppers), // 1

                Motors.scale(Motors.withEncoder(hardwareMap.dcMotor.get("frontRight"), false, true, stoppers), scaleFactor), // 2
                Motors.scale(Motors.withEncoder(hardwareMap.dcMotor.get("backRight"), true, true, stoppers), scaleFactor), // 3
                true, MAX_ROBOT_SPEED,MAX_ROBOT_SPEED_SIDEWAYS));

        servos = new Servos(ServoCfg.createServoMap(hardwareMap, servoStartPresetMap));

        gyro0 = new IMUGyro(hardwareMap.get(BNO055IMU.class, "imu0"));

        blockCollector = new BlockCollector(
                Motors.withoutEncoder(hardwareMap.dcMotor.get("collectorMotor"), false, false, stoppers)
        );

        liftArmV2 = new LiftArmV2(
                getWrist(),
                getGripper(),
                getFingerRight(),
                getFingerLeft(),
                Motors.withEncoder(hardwareMap.dcMotor.get("VerticalRightMotor"), false, true, stoppers),
                Motors.withEncoder(hardwareMap.dcMotor.get("VerticalLeftMotor"), true, true, stoppers),
                Motors.withEncoder(hardwareMap.dcMotor.get("HorizontalMotor"), true, true, stoppers),
                Sensors.inv(Sensors.digital(hardwareMap,"lowerLimitVerticalRight")),
                Sensors.inv(Sensors.digital(hardwareMap,"lowerLimitVerticalLeft")),
                Sensors.inv(Sensors.digital(hardwareMap,"lowerLimitHorizontal"))
        );

        newFoundationMover = new NewFoundationMover(
                getRightFoundationMover(),
                getLeftFoundationMover()
        );

      //  plusXDistanceSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "plusXSensor");


        minusYDistanceSensor = (Rev2mDistanceSensor)hardwareMap.get(DistanceSensor.class, "minusYSensor");
        plusYDistanceSensor = (Rev2mDistanceSensor)hardwareMap.get(DistanceSensor.class, "plusYSensor");

//        Function podsCal = new Function() {
//            @Override
//            public double f(double x) {
//                return (-2.04 + 2.15 * x - 0.0318*x*x); //CENTIMETERS!!!!!!
//            }
//        };
//        final ftc.evlib.hardware.sensors.AnalogSensor analogSensorRawPods;
//        analogSensorRawPods = Sensors.analog(hardwareMap, "plusYSensor");

//        plusYDistanceSensor = new AveragedSensor(analogSensorRawPods, 1, podsCal);
    }

    private final Servos servos;

    public SkystoneRobotCfgV2(HardwareMap hardwareMap) {
        this(hardwareMap, ServoCfg.defaultServoStartPresetMap(SkystoneServoName.values()));
    }

    public enum WristServoPresets {
        ZERO,
        NINETY
    }

    public enum GripperServoPresets {
        GRAB,
        RELEASE
    }

    public enum FingerLeftServoPresets {
        STOP,
        FORWARD,
        BACKWARD

    }
    public enum FingerRightServoPresets {
        STOP,
        FORWARD,
        BACKWARD
    }

    public enum RightFoundationMoverServoPresets {
        UP,
        READY,
        DOWN,
    }

    public enum LeftFoundationMoverServoPresets {
        UP,
        READY,
        DOWN,
    }


    public ServoControl getGripper() {
        return getServo(SkystoneServoName.GRIPPER_SERVO);
    }

    private final MecanumControl mecanumControl;

    public ServoControl getWrist() {
        return getServo(SkystoneServoName.WRIST_SERVO);
    }

    private static final Velocity MAX_ROBOT_SPEED = new Velocity(Distance.fromInches(57 * 4), Time.fromSeconds(2.83));
    private static final Velocity MAX_ROBOT_SPEED_SIDEWAYS = new Velocity(Distance.fromInches(21.2441207039), Time.fromSeconds(1));

    public ServoControl getFingerLeft() {
        return getServo(SkystoneServoName.FINGER_LEFT_SERVO);
    }

    public ServoControl getFingerRight() {
        return getServo(SkystoneServoName.FINGER_RIGHT_SERVO);
    }

    public ServoControl getRightFoundationMover() {
        return getServo(SkystoneServoName.RIGHT_FOUNDATION_MOVER_SERVO);
    }

    @Override
    public void start() {

    }

    @Override
    public void pre_act() {
        liftArmV2.pre_act();
    }

    @Override
    public void act() {
        mecanumControl.act();
        blockCollector.act();
        liftArmV2.act();
//        plusYDistanceSensor.act(); // only needed for average sensor
    }

    @Override
    public void stop() {
        mecanumControl.stop();
        blockCollector.stop();
        liftArmV2.stop();
        gyro0.stop();
    }

    public Servos getServos(){
        return servos;
    }

    public ServoControl getLeftFoundationMover() {
        return getServo(SkystoneServoName.LEFT_FOUNDATION_MOVER_SERVO);
    }

    public MecanumControl getMecanumControl() {
        return mecanumControl;
    }

    public NewFoundationMover getNewFoundationMover() {
        return newFoundationMover;
    }

    public Gyro getGyro() {
        return gyro0;
    }

    public BlockCollector getBlockCollector() {
        return blockCollector;
    }



    public LiftArmV2 getLiftArmV2() {
        return liftArmV2;
    }



//    public ModernRoboticsI2cRangeSensor getPlusXDistanceSensor() {
//        return plusXDistanceSensor;
//    }

    public Rev2mDistanceSensor getMinusYDistanceSensor() {
        return minusYDistanceSensor;
    }

    public Rev2mDistanceSensor getPlusYDistanceSensor() {
        return plusYDistanceSensor;
    }

    /**
     * defines all the servos on the robot
     */
    public enum SkystoneServoName implements ServoName {
        //enum name("hardware name", preset enum.values()),
        WRIST_SERVO("wristServo", WristServoPresets.values()),
        GRIPPER_SERVO("gripperServo", GripperServoPresets.values()),
        FINGER_LEFT_SERVO("fingerLeftServo", FingerLeftServoPresets.values()),
        FINGER_RIGHT_SERVO("fingerRightServo", FingerRightServoPresets.values()),
        RIGHT_FOUNDATION_MOVER_SERVO("rightFoundationMoverServo", RightFoundationMoverServoPresets.values()),
        LEFT_FOUNDATION_MOVER_SERVO("leftFoundationMoverServo", LeftFoundationMoverServoPresets.values());

//        PUSH_SERVO("pushServo", RotateServoPresets.values());

        private final String hardwareName;
        private final Enum[] presets;

        SkystoneServoName(String hardwareName, Enum[] presets) {
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
        public Class<SkystoneRobotCfgV2> getRobotCfg() {
            return SkystoneRobotCfgV2.class;
        }
    }
}
