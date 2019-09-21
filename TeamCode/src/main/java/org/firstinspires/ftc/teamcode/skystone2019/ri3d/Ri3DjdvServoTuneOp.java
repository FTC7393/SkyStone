package org.firstinspires.ftc.teamcode.skystone2019.ri3d;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.futurefest2019.FutureFestRobotCfg;

import ftc.evlib.hardware.config.RobotCfg;
import ftc.evlib.opmodes.AbstractServoTuneOp;

@TeleOp(name = "Ri3DServoTuneOp")
public class Ri3DjdvServoTuneOp extends AbstractServoTuneOp {

    protected RobotCfg createRobotCfg() {
        //create a new SampleRobotConfig and return it.
        //the superclass will extract the servos and do the rest.
        return new Ri3DjdvRobotCfg(hardwareMap);
    }
}
