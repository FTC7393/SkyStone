package org.firstinspires.ftc.teamcode.Abhi_Robot;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import ftc.evlib.hardware.config.RobotCfg;
import ftc.evlib.opmodes.AbstractServoTuneOp;

@TeleOp(name = "TankDriveServoTuneOp")
public class TankDriveServoTuneOp extends AbstractServoTuneOp {

    protected RobotCfg createRobotCfg() {
        //create a new robot config and return it.
        //the superclass will extract the servos and do the rest.
        return new TankDriveRobotCfg(hardwareMap);
    }
}
