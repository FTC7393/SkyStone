package org.firstinspires.ftc.teamcode.SkyStone_Season;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SkyStone_Season.Autonomous.SkystoneRobotCfg;

import ftc.evlib.hardware.config.RobotCfg;
import ftc.evlib.opmodes.AbstractServoTuneOp;

@TeleOp(name = "Skystone JDV ServoTuneOp")
@Disabled
public class SkystoneServoTuneOp extends AbstractServoTuneOp {

    protected RobotCfg createRobotCfg() {
        //create a new SampleRobotConfig and return it.
        //the superclass will extract the servos and do the rest.
        return new SkystoneRobotCfg(hardwareMap);
    }
}
//comment