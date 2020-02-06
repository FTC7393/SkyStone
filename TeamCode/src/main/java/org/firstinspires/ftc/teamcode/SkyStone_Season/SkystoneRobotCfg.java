package org.firstinspires.ftc.teamcode.SkyStone_Season;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.SkyStone_Season.TeleOp.FlyWheels;
import org.firstinspires.ftc.teamcode.SkyStone_Season.TeleOp.FoundationMover;
import org.firstinspires.ftc.teamcode.SkyStone_Season.TeleOp.LiftArm;

import java.util.Map;

import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.util.Function;
import ftc.electronvolts.util.TeamColor;
import ftc.electronvolts.util.units.Angle;
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
import ftc.evlib.statemachine.EVStateMachineBuilder;
//comment
/**
 * Created by ftc7393 on 9/22/2018.
 */

public class SkystoneRobotCfg extends RobotCfg {

    private final FlyWheels flyWheels;
    private final Gyro gyro1;
    private Gyro gyro0;
    private final LiftArm liftArm;
    private final FoundationMover foundationMover;
    private final ModernRoboticsI2cRangeSensor plusXDistanceSensor;
    private final ModernRoboticsI2cRangeSensor minusXDistanceSensor;
    private final AveragedSensor plusYDistanceSensor;



    public SkystoneRobotCfg(HardwareMap hardwareMap, Map<ServoName, Enum> servoStartPresetMap) {
        super(hardwareMap);
        double scaleFactor = 1.0;
        mecanumControl = new MecanumControl(new MecanumMotors(
                Motors.withEncoder(hardwareMap.get(DcMotorEx.class, "frontLeft"), true, true, stoppers), // 0
                Motors.withEncoder(hardwareMap.get(DcMotorEx.class, "frontRight"), false, true, stoppers), // 1

                Motors.scale(Motors.withEncoder(hardwareMap.get(DcMotorEx.class, "backRight"), true, true, stoppers), scaleFactor), // 2
                Motors.scale(Motors.withEncoder(hardwareMap.get(DcMotorEx.class,"backLeft"), false, true, stoppers), scaleFactor), // 3
                true, MAX_ROBOT_SPEED,MAX_ROBOT_SPEED_SIDEWAYS));

        servos = new Servos(ServoCfg.createServoMap(hardwareMap, servoStartPresetMap));

        gyro0 = new IMUGyro(hardwareMap.get(BNO055IMU.class, "imu0"));
        gyro1 = new IMUGyro(hardwareMap.get(BNO055IMU.class, "imu1"));

        flyWheels = new FlyWheels(
                Motors.withoutEncoder(hardwareMap.dcMotor.get("leftFlywheel"), false, false, stoppers),
                Motors.withoutEncoder(hardwareMap.dcMotor.get("rightFlywheel"), false, false, stoppers)

        );

        liftArm = new LiftArm(
                getElbow(),
                getWrist(),
                getFingers(),
                Motors.withEncoder(hardwareMap.dcMotor.get("extension"), false, true, stoppers),
                Sensors.inv(Sensors.digital(hardwareMap,"lowerLimit")));

        foundationMover = new FoundationMover(
                getRightFoundationMover(),
                getLeftFoundationMover()
        );

        plusXDistanceSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "plusXSensor");

        minusXDistanceSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "minusXSensor");

        Function podsCal = new Function() {
            @Override
            public double f(double x) {
                return (-2.04 + 2.15 * x - 0.0318*x*x); //CENTIMETERS!!!!!!
            }
        };
        final ftc.evlib.hardware.sensors.AnalogSensor analogSensorRawPods;
        analogSensorRawPods = Sensors.analog(hardwareMap, "pods");

        plusYDistanceSensor = new AveragedSensor(analogSensorRawPods, 1, podsCal);
    }

    private final Servos servos;

    public SkystoneRobotCfg(HardwareMap hardwareMap) {
        this(hardwareMap, ServoCfg.defaultServoStartPresetMap(SkystoneServoName.values()));
    }

    public enum ElbowServoPresets {
        STOWED,
        GRABBING,
        PLACING
    }

    public enum WristServoPresets {
        STOWED,
        GRABBING,
        PLACING,
        PLACING_LEFT
    }

    public enum FingersServoPresets {
        GRAB,
        RELEASE
    }

    public enum SideArmPresets {
        STOWED,
        GRABBING,
        CARRY,
        PREDRIVE
    }
    public enum SideGrabberPresets {
        OPEN,
        CLOSED
    }


    public ServoControl getElbow() {
        return getServo(SkystoneServoName.ELBOW_SERVO);
    }


    private final MecanumControl mecanumControl;

    public ServoControl getWrist() {
        return getServo(SkystoneServoName.WRIST_SERVO);
    }

    private static final Velocity MAX_ROBOT_SPEED = new Velocity(Distance.fromInches(57 * 4), Time.fromSeconds(2.83));
    private static final Velocity MAX_ROBOT_SPEED_SIDEWAYS = new Velocity(Distance.fromInches(21.2441207039), Time.fromSeconds(1));

    public ServoControl getFingers() {
        return getServo(SkystoneServoName.FINGERS_SERVO);
    }

    public ServoControl getRightFoundationMover() {
        return getServo(SkystoneServoName.RIGHT_FOUNDATION_MOVER_SERVO);
    }

    @Override
    public void start() {

    }

    @Override
    public void pre_act() {
        liftArm.pre_act();
    }

    @Override
    public void act() {
        mecanumControl.act();
        flyWheels.act();
        liftArm.act();
        plusYDistanceSensor.act();
    }

    @Override
    public void stop() {
        mecanumControl.stop();
        flyWheels.stop();
        liftArm.stop();
        gyro0.stop();
    }

    public Servos getServos(){
        return servos;
    }

    public ServoControl getLeftFoundationMover() {
        return getServo(SkystoneServoName.LEFT_FOUNDATION_MOVER_SERVO);
    }
    public ServoControl SideArm() {
        return getServo(SkystoneServoName.SIDEARM_SERVO);
    }
    public ServoControl SideGrabber() {
        return getServo(SkystoneServoName.SIDEGRABBER_SERVO);
    }


    public MecanumControl getMecanumControl() {
        return mecanumControl;
    }

    public FoundationMover getFoundationMover() {
        return foundationMover;
    }

    public ServoControl getSideArm() {
        return getServo(SkystoneServoName.SIDEARM_SERVO);
    }

    public ServoControl getSideGrabber() {
        return getServo(SkystoneServoName.SIDEGRABBER_SERVO);
    }


    public enum RightFoundationMoverServoPresets {
        UP,
        DOWN
    }

    public enum LeftFoundationMoverServoPresets {
        UP,
        DOWN
    }



    public Gyro getGyro() {
        return gyro0;
    }

    public FlyWheels getFlyWheels() {
        return flyWheels;
    }



    public LiftArm getLiftArm() {
        return liftArm;
    }



    public ModernRoboticsI2cRangeSensor getPlusXDistanceSensor() {
        return plusXDistanceSensor;
    }

    public ModernRoboticsI2cRangeSensor getMinusXDistanceSensor() {
        return minusXDistanceSensor;
    }

    public AveragedSensor getPlusYDistanceSensor() {
        return plusYDistanceSensor;
    }

    /**
     * defines all the servos on the robot
     */
    public enum SkystoneServoName implements ServoName {
        //enum name("hardware name", preset enum.values()),
        ELBOW_SERVO("elbowServo", ElbowServoPresets.values()),
        WRIST_SERVO("wristServo", WristServoPresets.values()),
        FINGERS_SERVO("fingersServo", FingersServoPresets.values()),
        RIGHT_FOUNDATION_MOVER_SERVO("rightFoundationMoverServo", RightFoundationMoverServoPresets.values()),
        LEFT_FOUNDATION_MOVER_SERVO("leftFoundationMoverServo", LeftFoundationMoverServoPresets.values()),
        SIDEARM_SERVO("stoneGrabberArm", SideArmPresets.values()),
        SIDEGRABBER_SERVO("stoneGrabberServo", SideGrabberPresets.values());

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
        public Class<SkystoneRobotCfg> getRobotCfg() {
            return SkystoneRobotCfg.class;
        }
    }
}
