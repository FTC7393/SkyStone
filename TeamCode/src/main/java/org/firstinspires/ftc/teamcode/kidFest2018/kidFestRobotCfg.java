package org.firstinspires.ftc.teamcode.kidFest2018;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import evlib.hardware.config.RobotCfg;
import evlib.hardware.control.MecanumControl;
import evlib.hardware.motors.MecanumMotors;
import evlib.hardware.motors.Motors;
import ftc.electronvolts.util.units.Distance;
import ftc.electronvolts.util.units.Time;
import ftc.electronvolts.util.units.Velocity;

/**
 * Created by ftc7393 on 9/22/2018.
 */

public class kidFestRobotCfg extends RobotCfg {
    private final MecanumControl mecanumControl;
    private DcMotor dump = null;
    private DcMotor collector = null;

    private static final Velocity MAX_ROBOT_SPEED = new Velocity(Distance.fromInches(57 * 4), Time.fromSeconds(2.83));
    private static final Velocity MAX_ROBOT_SPEED_SIDEWAYS = new Velocity(Distance.fromInches(21.2441207039), Time.fromSeconds(1));
    public kidFestRobotCfg(HardwareMap hardwareMap) {
        super(hardwareMap);
        double scaleFactor = 1.0;
        mecanumControl = new MecanumControl(new MecanumMotors(
                Motors.withEncoder(hardwareMap.dcMotor.get("frontLeft"), true, true, stoppers), // 0
                Motors.withEncoder(hardwareMap.dcMotor.get("frontRight") , false, true, stoppers), // 1
                Motors.scale(Motors.withEncoder(hardwareMap.dcMotor.get("backRight") , true, true, stoppers),scaleFactor), // 2
                Motors.scale(Motors.withEncoder(hardwareMap.dcMotor.get("backLeft") , false, true, stoppers),scaleFactor), // 3
                true, MAX_ROBOT_SPEED,MAX_ROBOT_SPEED_SIDEWAYS));
        dump  = hardwareMap.get(DcMotor.class, "arm");
        dump.setPower(0);
        dump.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        collector  = hardwareMap.get(DcMotor.class, "collection");
        collector.setPower(0);
        collector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }
    @Override
    public void start() {

    }

    @Override
    public void act() {
        mecanumControl.act();

    }

    @Override
    public void stop() {
        mecanumControl.stop();


    }
    public DcMotor getDump(){return dump;}
    public DcMotor getCollector(){return collector;}
    public MecanumControl getMecanumControl() {
        return mecanumControl;
    }

}
