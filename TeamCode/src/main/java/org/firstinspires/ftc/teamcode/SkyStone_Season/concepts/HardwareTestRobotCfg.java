package org.firstinspires.ftc.teamcode.SkyStone_Season.concepts;

import com.qualcomm.robotcore.hardware.HardwareMap;

import ftc.electronvolts.util.units.Distance;
import ftc.electronvolts.util.units.Time;
import ftc.electronvolts.util.units.Velocity;
import ftc.evlib.hardware.config.RobotCfg;
import ftc.evlib.hardware.motors.Motors;
import ftc.evlib.hardware.motors.TwoMotors;
import ftc.evlib.hardware.servos.ServoName;

/**
 * Created by ftc7393 on 9/22/2018.
 */

public class HardwareTestRobotCfg extends RobotCfg {

private final TwoMotors twoMotors;

private static final Velocity maxRobotSpeed = new Velocity(Distance.fromFeet(1), Time.fromSeconds(1));


    public HardwareTestRobotCfg(HardwareMap hardwareMap) { // }, Map<ServoName, Enum> servoStartPresetMap) {
        super(hardwareMap);


       twoMotors = new TwoMotors(
               Motors.withoutEncoder(hardwareMap.dcMotor.get("leftmotor"), false, false, stoppers),
               Motors.withoutEncoder(hardwareMap.dcMotor.get("rightmotor"), false, false, stoppers),
               false, maxRobotSpeed
       );

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
            return HardwareTestRobotCfg.class;
        }
    }



    @Override
    public void start() {

    }




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

}


