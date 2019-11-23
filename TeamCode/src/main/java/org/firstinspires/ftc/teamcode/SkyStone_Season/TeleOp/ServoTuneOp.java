package org.firstinspires.ftc.teamcode.SkyStone_Season.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SkyStone_Season.SkystoneRobotCfg;
import ftc.evlib.hardware.config.RobotCfg;
import ftc.evlib.opmodes.AbstractServoTuneOp;


    @TeleOp(name = "SkystoneServoTuneOp")

    public class ServoTuneOp extends AbstractServoTuneOp {

        @Override
        protected RobotCfg createRobotCfg() { return new SkystoneRobotCfg(hardwareMap); }



    }


