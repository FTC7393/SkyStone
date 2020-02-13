package org.firstinspires.ftc.teamcode.SkyStone_Season.RobotV2;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SkyStone_Season.SkystoneRobotCfg;

import ftc.evlib.hardware.config.RobotCfg;
import ftc.evlib.opmodes.AbstractServoTuneOp;


@TeleOp(name = "SkystoneServoTuneOp")

public class ServoTuneOpV2 extends AbstractServoTuneOp {

    @Override
    protected SkystoneRobotCfgV2 createRobotCfg() { return new SkystoneRobotCfgV2(hardwareMap); }

}


