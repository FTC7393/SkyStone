package org.firstinspires.ftc.teamcode.SkyStone_Season;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.SkyStone_Season.TeleOp.FlyWheels;
import org.firstinspires.ftc.teamcode.SkyStone_Season.TeleOp.FoundationMover;
import org.firstinspires.ftc.teamcode.SkyStone_Season.TeleOp.LiftArm;

import java.util.Map;

import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.util.TeamColor;
import ftc.electronvolts.util.units.Angle;
import ftc.electronvolts.util.units.Distance;
import ftc.electronvolts.util.units.Time;
import ftc.electronvolts.util.units.Velocity;
import ftc.evlib.hardware.config.RobotCfg;
import ftc.evlib.hardware.control.MecanumControl;
import ftc.evlib.hardware.motors.MecanumMotors;
import ftc.evlib.hardware.motors.Motors;
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
    private Gyro gyro;
    private final LiftArm liftArm;
    private final FoundationMover foundationMover;

    public enum ElbowServoPresets {
        RETRACT,
        EXTEND
    }

    public enum WristServoPresets {
        RETRACT,
        EXTEND,
        RIGHT,
        LEFT
    }

    public SkystoneRobotCfg(HardwareMap hardwareMap, Map<ServoName, Enum> servoStartPresetMap) {
        super(hardwareMap);
        double scaleFactor = 1.0;
        mecanumControl = new MecanumControl(new MecanumMotors(
                Motors.withEncoder(hardwareMap.dcMotor.get("frontLeft"), true, true, stoppers), // 0
                Motors.withEncoder(hardwareMap.dcMotor.get("frontRight"), false, true, stoppers), // 1

                Motors.scale(Motors.withEncoder(hardwareMap.dcMotor.get("backRight"), true, true, stoppers), scaleFactor), // 2
                Motors.scale(Motors.withEncoder(hardwareMap.dcMotor.get("backLeft"), false, true, stoppers), scaleFactor), // 3
                true, MAX_ROBOT_SPEED,MAX_ROBOT_SPEED_SIDEWAYS));
//        collector  = hardwareMap.get(DcMotor.class, "collection");
//        collector.setPower(0);
//        collector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
        servos = new Servos(ServoCfg.createServoMap(hardwareMap, servoStartPresetMap));

        gyro = new IMUGyro(hardwareMap.get(BNO055IMU.class, "imu"));

        flyWheels = new FlyWheels(
                Motors.withoutEncoder(hardwareMap.dcMotor.get("leftFlywheel"), false, false, stoppers),
                Motors.withoutEncoder(hardwareMap.dcMotor.get("rightFlywheel"), false, false, stoppers)
//                Motors.withoutEncoder(hardwareMap.dcMotor.get("frontLeft"), false, false, stoppers),
//                Motors.withoutEncoder(hardwareMap.dcMotor.get("frontRight"), false, false, stoppers)

        );

        liftArm = new LiftArm(
                getElbow(),
                getWrist(),
                getFingers(),
                Motors.withEncoder(hardwareMap.dcMotor.get("extension"), true, true, stoppers),
                Sensors.inv(Sensors.digital(hardwareMap,"lowerLimit")),
                Sensors.inv(Sensors.digital(hardwareMap,"upperLimit"))
        );

        foundationMover = new FoundationMover(
                getRightFoundationMover(),
                getLeftFoundationMover()
        );

    }

    private final Servos servos;

    public SkystoneRobotCfg(HardwareMap hardwareMap) {
        //this(hardwareMap, ServoCfg.defaultServoStartPresetMap(FutureFestServoEnum.values()));
        this(hardwareMap, ServoCfg.defaultServoStartPresetMap(SkystoneServoName.values()));
    }

    public enum FingersServoPresets {
        GRAB,
        RELEASE
    }

//    public enum pushServoPresets {
//        RETRACT,
//        EJECT
//    }

    public ServoControl getElbow() {
        return getServo(SkystoneServoName.ELBOW_SERVO);
    }


    private final MecanumControl mecanumControl;

    public ServoControl getWrist() {
        return getServo(SkystoneServoName.WRIST_SERVO);
    }
//    private DcMotor collector = null;

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
    public void act() {
        mecanumControl.act();
        flyWheels.act();
        liftArm.act();
        servos.act();
    }

    @Override
    public void stop() {
        mecanumControl.stop();
        flyWheels.stop();
        gyro.stop();
    }

    public Servos getServos(){
        return servos;
    }

    public ServoControl getLeftFoundationMover() {
        return getServo(SkystoneServoName.LEFT_FOUNDATION_MOVER_SERVO);
    }

    //    public DcMotor getCollector(){
//        return collector;
//    }
    public MecanumControl getMecanumControl() {
        return mecanumControl;
    }

    public FoundationMover getFoundationMover() {
        return foundationMover;
    }

    public enum RightFoundationMoverServoPresets {
        UP,
        DOWN
    }

    public enum LeftFoundationMoverServoPresets {
        UP,
        DOWN
    }

    public enum StoneScraperServoPresets {
        UP,
        DOWN
    }


    public Gyro getGyro() {
        return gyro;
    }

    public FlyWheels getFlyWheels() {
        return flyWheels;
    }

    public EVStateMachineBuilder createEVStateMachineBuilder(StateName firstStateName, TeamColor teamColor, Angle tolerance) {
        return new EVStateMachineBuilder(firstStateName, teamColor, tolerance, gyro, servos, mecanumControl);
    }

    public LiftArm getLiftArm() {
        return liftArm;
    }

    public ServoControl getStoneScraperServo() {
        return getServo(SkystoneServoName.STONE_SCRAPER_SERVO);
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
        STONE_SCRAPER_SERVO("stoneScraperServo", StoneScraperServoPresets.values());

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