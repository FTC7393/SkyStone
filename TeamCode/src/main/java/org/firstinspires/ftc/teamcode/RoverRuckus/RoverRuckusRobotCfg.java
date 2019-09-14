package org.firstinspires.ftc.teamcode.RoverRuckus;

import android.util.Log;

import com.google.common.collect.ImmutableList;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;
import java.util.Map;

import ftc.evlib.hardware.config.RobotCfg;
import ftc.evlib.hardware.control.MecanumControl;
import ftc.evlib.hardware.motors.MecanumMotors;
import ftc.evlib.hardware.motors.MotorEnc;
import ftc.evlib.hardware.motors.Motors;
import ftc.evlib.hardware.sensors.Gyro;
import ftc.evlib.hardware.sensors.IMUGyro;
import ftc.evlib.hardware.sensors.Sensors;
import ftc.evlib.hardware.servos.ServoCfg;
import ftc.evlib.hardware.servos.ServoControl;
import ftc.evlib.hardware.servos.ServoName;
import ftc.evlib.hardware.servos.Servos;
import ftc.evlib.statemachine.EVStateMachineBuilder;
import ftc.evlib.util.StepTimer;
import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.util.InputExtractor;
import ftc.electronvolts.util.TeamColor;
import ftc.electronvolts.util.files.Logger;
import ftc.electronvolts.util.units.Angle;
import ftc.electronvolts.util.units.Distance;
import ftc.electronvolts.util.units.Time;
import ftc.electronvolts.util.units.Velocity;

import static ftc.evlib.vision.framegrabber.GlobalFrameGrabber.frameGrabber;

/**
 * Created by ftc7393 on 9/29/2018.
 */

public class RoverRuckusRobotCfg extends RobotCfg {
    //private final ColorSensor leftColorSensor;
    //private final ColorSensor rightColorSensor;
    private final MecanumControl mecanumControl;
    private final Collector collector;
    private final Hanging hanging;
    private final ServoControl marker;
    private final ServoControl phonePan;
    final MotorEnc frontLeft;
    final MotorEnc frontRight;
    final MotorEnc backLeft;
    final MotorEnc backRight;
    private final ServoControl doorRight;
    private final ServoControl doorLeft;


    //    private final Collector collector;
    private final Arm arm;
    private static final Velocity MAX_ROBOT_SPEED = new Velocity(Distance.fromInches(57 * 4), Time.fromSeconds(2.83));
    private static final Velocity MAX_ROBOT_SPEED_SIDEWAYS = new Velocity(Distance.fromInches(21.2441207039), Time.fromSeconds(1));
    private static final String LEFT_COLOR_SENSOR_NAME = "colorBackLeft";
    private static final String RIGHT_COLOR_SENSOR_NAME = "colorBackRight";
    private IMUGyro gyro;
    private final Servos servos;
    ServoControl backFoot;

    public MotorEnc getTestMotor() {
        return backLeft;
    }

    public RoverRuckusRobotCfg(HardwareMap hardwareMap, Map<ServoName, Enum> servoStartPresetMap) {
        super(hardwareMap);
        double scaleFactor = 1.0;

        servos = new Servos(ServoCfg.createServoMap(hardwareMap, servoStartPresetMap));
        doorRight =   getServo(MainServoName.DOORRIGHT);
        doorLeft =   getServo(MainServoName.DOORLEFT);
        marker = getServo(MainServoName.MARKER);
        phonePan=getServo(MainServoName.PHONEPAN);



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

        arm=new Arm(
                Motors.withEncoder(hardwareMap.dcMotor.get("extension"),false,true,stoppers),
                Sensors.inv(Sensors.digital(hardwareMap,"extensionLimit")),
                Motors.withEncoder(hardwareMap.dcMotor.get("rotation"),false,true,stoppers),
                Sensors.analog(hardwareMap,"potentiometer"));
        collector=new Collector(
                Motors.withoutEncoder(hardwareMap.dcMotor.get("collection"),false,true,stoppers),
                getServo(MainServoName.DOORRIGHT),
                getServo(MainServoName.DOORLEFT)        );
         backFoot=getServo(MainServoName.BACKFOOT);

         hanging=new Hanging(
                 Motors.withoutEncoder(hardwareMap.dcMotor.get("hanging"),true,true,stoppers),
                 getServo(MainServoName.LATCH),
                 Sensors.inv(Sensors.digital(hardwareMap,"latchLimit")),
                 Sensors.inv(Sensors.digital(hardwareMap,"unlatchLimit"))
         );




        loggerColumns = ImmutableList.of(
                new Logger.Column("Rotation Encoder",new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return getArm().getRotationEncoder();
                    }
                }),
                new Logger.Column("Rotation SetPoint",new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return getArm().getRotationSetPoint();
                    }
                }),
                new Logger.Column("Rotation Min Encoder Value",new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return getArm().getMinRotationValue();
                    }
                }),

                new Logger.Column("Rotation Power Output",new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return getArm().getRotationPower();
                    }
                }),
                new Logger.Column("Extension Encoder",new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return getArm().getExtensionEncoder();
                    }
                }),
                new Logger.Column("Extension SetPoint",new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return getArm().getExtensionSetPoint();
                    }
                }),
                new Logger.Column("Extension Min Encoder Value",new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return getArm().getMinExtensionValue();
                    }
                }),

                new Logger.Column("Extension Power Output",new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return getArm().getExtensionPower();
                    }
                }),

                new Logger.Column("Potentiometer",new InputExtractor<Double>() {
                @Override
                public Double getValue() {
                    return getArm().getPotentiometerValue();
                    }
                }),
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



//
//
//
// new Logger.Column("Extension Limit Switch",new InputExtractor<Boolean>() {
//                    @Override
//                    public Boolean getValue() {
//                        return getArm().getExtensionLimitSwitch();
//                    }
//                })


        );



//hi




    }
    public enum DoorRightPresets {
        CLOSE,
        OPEN
    }

    public enum DoorLeftPresets {
        CLOSE,
        OPEN
    }
    public enum BackFootPresets{
        LOCKED,
        OPENED,
        DEPLOYED
    }

    public enum LatchPresets{
        LATCH,
        UNLATCH
    }
    public enum MarkerPresets{
        HOLD,
        RELEASE
    }
    public enum PhonePanPresets{
        MIDDLE,
        LEFT,
        RIGHT,
        STOW

    }
    public enum MainServoName implements ServoName {
        LATCH("latch", LatchPresets.values()),
        DOORRIGHT("doorRight", DoorRightPresets.values()),
        DOORLEFT("doorLeft", DoorLeftPresets.values()),
        BACKFOOT("backFoot", BackFootPresets.values()),
        MARKER("marker", MarkerPresets.values()),
        PHONEPAN("phonePan", PhonePanPresets.values());

        private final String hardwareName;
        private final Enum[] presets;

        MainServoName(String hardwareName, Enum[] presets) {
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
            return RoverRuckusRobotCfg.class;
        }
    }
    public RoverRuckusRobotCfg(HardwareMap hardwareMap){
        this(hardwareMap, ServoCfg.defaultServoStartPresetMap(MainServoName.values()));
    }
    //public ColorSensor getLeftColorSensor() {
    //    return leftColorSensor;
    //}
    //public ColorSensor getRightColorSensor() {
    //    return rightColorSensor;
    //}
    public Gyro getGyro() {
        return gyro;
    }


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

        myTimer.step("arm.act");
        arm.act();
        myTimer.step("collector.act");
        collector.act();
        myTimer.step("hanging.act");
        hanging.act();
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
    public Arm getArm(){
        return arm;
    }

    public Hanging getHanging() {
        return hanging;
    }

    public Collector getCollector(){
        return collector;
    }
    public ServoControl getBackFoot(){return backFoot;}
    public ServoControl getMarker(){return marker;}
    public ServoControl getPhonePan(){return phonePan;}

    @Override
    public Servos getServos() {
        return servos;
    }
    private final List<Logger.Column> loggerColumns;

    public List<Logger.Column> getLoggerColumns() {
        return loggerColumns;
    }
    public EVStateMachineBuilder createEVStateMachineBuilder(StateName firstStateName, TeamColor teamColor, Angle tolerance) {
        return new EVStateMachineBuilder(firstStateName, teamColor, tolerance, gyro, frameGrabber, servos, mecanumControl);
    }

}
