package org.firstinspires.ftc.teamcode.fair2019;

/**
 * Created by ftc7393 on 7/31/2018.
 */

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import evlib.hardware.config.RobotCfg;
import evlib.hardware.motors.Motors;
import evlib.hardware.motors.TwoMotors;
import ftc.electronvolts.util.units.Distance;
import ftc.electronvolts.util.units.Time;
import ftc.electronvolts.util.units.Velocity;

/**
 * This file was made by the electronVolts, FTC team 7393
 * Date Created: 9/12/16
 * <p>
 * The HardwareCfg for the simple fair parade robot with 2 motors.
 */
public class Fair2019RobotCfg extends RobotCfg {
    /**
     * The speed the robot moves at when the power is set to 100%
     */
    private static final Velocity maxRobotSpeed = new Velocity(Distance.fromFeet(1), Time.fromSeconds(1));


    private final TwoMotors twoMotors;
    private DcMotor chest = null;


    public Fair2019RobotCfg(HardwareMap hardwareMap) {
        super(hardwareMap);

        //get the two motors from the hardwareMap and put them into a TwoMotors object
        twoMotors = new TwoMotors(
                Motors.withoutEncoder(hardwareMap.dcMotor.get("leftmotor"), false, false, stoppers),
                Motors.withoutEncoder(hardwareMap.dcMotor.get("rightmotor"), true, false, stoppers),
                false, maxRobotSpeed
        );
        chest = hardwareMap.get(DcMotor.class, "lidmotor");
        chest.setPower(0);
        chest.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
    public DcMotor getChest(){return chest;}
}
