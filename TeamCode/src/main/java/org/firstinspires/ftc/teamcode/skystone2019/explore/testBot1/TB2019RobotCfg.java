package org.firstinspires.ftc.teamcode.skystone2019.explore.testBot1;

import android.util.Log;

import com.google.common.collect.ImmutableList;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.util.InputExtractor;
import ftc.electronvolts.util.TeamColor;
import ftc.electronvolts.util.files.Logger;
import ftc.electronvolts.util.units.Angle;
import ftc.electronvolts.util.units.Distance;
import ftc.electronvolts.util.units.Time;
import ftc.electronvolts.util.units.Velocity;
import ftc.evlib.hardware.config.RobotCfg;
import ftc.evlib.hardware.control.MecanumControl;
import ftc.evlib.hardware.motors.MecanumMotors;
import ftc.evlib.hardware.motors.MotorEnc;
import ftc.evlib.hardware.motors.Motors;
import ftc.evlib.hardware.sensors.Gyro;
import ftc.evlib.hardware.sensors.IMUGyro;
import ftc.evlib.hardware.servos.ServoCfg;
import ftc.evlib.hardware.servos.ServoControl;
import ftc.evlib.hardware.servos.ServoName;
import ftc.evlib.hardware.servos.Servos;
import ftc.evlib.statemachine.EVStateMachineBuilder;
import ftc.evlib.util.StepTimer;



class TB2019RobotCfg  extends RobotCfg {
    //private final ColorSensor leftColorSensor;
    //private final ColorSensor rightColorSensor;
    private final MecanumControl mecanumControl;
//    private final ServoControl phonePan;
    private final MotorEnc frontLeft;
    private final MotorEnc frontRight;
    private final MotorEnc backLeft;
    private final MotorEnc backRight;

    private static final Velocity MAX_ROBOT_SPEED = new Velocity(Distance.fromInches(57 * 4), Time.fromSeconds(2.83));
    private static final Velocity MAX_ROBOT_SPEED_SIDEWAYS = new Velocity(Distance.fromInches(21.2441207039), Time.fromSeconds(1));


    private final CameraPlacement cameraPlacement;
    private IMUGyro gyro;
    private final Servos servos;

    public MotorEnc getTestMotor() {
        return backLeft;
    }

    public TB2019RobotCfg(HardwareMap hardwareMap, CameraPlacement cameraPlacement) { // }, Map<ServoName, Enum> servoStartPresetMap) {
        super(hardwareMap);
        double scaleFactor = 1.0;

        this.cameraPlacement = cameraPlacement;
        servos = new Servos(new HashMap<ServoName, ServoControl>());
//        servos = new Servos(ServoCfg.createServoMap(hardwareMap, servoStartPresetMap));
//        phonePan=getServo(TBServos.PHONEPAN);



        //leftColorSensor = hardwareMap.colorSensor.get(LEFT_COLOR_SENSOR_NAME);
        //rightColorSensor = hardwareMap.colorSensor.get(RIGHT_COLOR_SENSOR_NAME);
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


        loggerColumns = ImmutableList.of(
                new Logger.Column("VeloxityX",new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return getMecanumControl().getVelocityX();
                    }
                }),
                new Logger.Column("VeloxityY",new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return getMecanumControl().getVelocityY();
                    }
                }),new Logger.Column("VeloxityR",new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return getMecanumControl().getVelocityR();
                    }
                })
        );



//hi




    }
    public enum PhonePanServoPresets{
        MIDDLE,
        LEFT,
        RIGHT,
        STOW
    }
    public enum TBServos implements ServoName {
        PHONEPAN("phonePan", PhonePanServoPresets.values());

        private final String hardwareName;
        private final Enum[] presets;

        TBServos(String hardwareName, Enum[] presets) {
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
        public Class<? extends RobotCfg> getRobotCfg() {
            return TB2019RobotCfg.class;
        }
    }
//    public TB2019RobotCfg(HardwareMap hardwareMap, CameraPlacement cameraPlacement){
//        this(hardwareMap, cameraPlacement); // , ServoCfg.defaultServoStartPresetMap(TBServos.values()));
//    }


    @Override
    public void start() {

    }

    StepTimer myTimer = new StepTimer("robotCfg", Log.INFO);


    @Override
    public void act() {
        myTimer.start();
        myTimer.step("mecanumControl.act");
        mecanumControl.act();
        // when testing the motors, uncomment these (and comment out mecanum.act()
//        frontLeft.update();
//        frontRight.update();
//        backLeft.update();
//        backRight.update();

        myTimer.stop();
    }

    @Override
    public void stop() {
        mecanumControl.stop();
        gyro.stop();

    }
    public MecanumControl getMecanumControl() {
        return mecanumControl;
    }
//    public ServoControl getPhonePan(){return phonePan;}
    public CameraPlacement getCameraPlacement() {
        return cameraPlacement;
    }
    @Override
    public Servos getServos() {
        return servos;
    }
    private final List<Logger.Column> loggerColumns;
    public Gyro getGyro() {
        return gyro;
    }

    public List<Logger.Column> getLoggerColumns() {
        return loggerColumns;
    }
    public EVStateMachineBuilder createEVStateMachineBuilder(StateName firstStateName, TeamColor teamColor, Angle tolerance) {
        return new EVStateMachineBuilder(firstStateName, teamColor, tolerance, gyro, servos, mecanumControl);
    }

}
