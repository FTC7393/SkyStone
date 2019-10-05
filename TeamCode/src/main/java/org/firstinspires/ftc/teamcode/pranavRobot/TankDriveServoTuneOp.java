package org.firstinspires.ftc.teamcode.pranavRobot;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.skystone2019.ri3d.Ri3DjdvRobotCfg;

import ftc.evlib.hardware.config.RobotCfg;
import ftc.evlib.opmodes.AbstractServoTuneOp;

@TeleOp(name = "TankDriveServoTuneOp")
public class TankDriveServoTuneOp extends AbstractServoTuneOp {

    protected RobotCfg createRobotCfg() {
        //create a new robot config and return it.
        //the superclass will extract the servos and do the rest.
        return new TankDrive2019RobotCfg(hardwareMap);
    }
}
