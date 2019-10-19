package org.firstinspires.ftc.teamcode.Skystone.Ri3D;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.futurefest2019.FutureFestRobotCfg;

import ftc.evlib.hardware.config.RobotCfg;
import ftc.evlib.opmodes.AbstractServoTuneOp;

@TeleOp(name = "SkyStoneRi3DServoTuneOp JDV")
public class SkystoneRi3DServoTuneOp extends AbstractServoTuneOp {

    protected RobotCfg createRobotCfg() {
        //create a new SampleRobotConfig and return it.
        //the superclass will extract the servos and do the rest.
        return new SkystoneRi3DRobotCfg(hardwareMap);
    }
}
