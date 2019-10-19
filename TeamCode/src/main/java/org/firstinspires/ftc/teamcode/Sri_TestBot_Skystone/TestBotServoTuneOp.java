package org.firstinspires.ftc.teamcode.Sri_TestBot_Skystone;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Abhi_Robot.TestBotCfg;

import ftc.evlib.hardware.config.RobotCfg;
import ftc.evlib.opmodes.AbstractServoTuneOp;

@TeleOp(name = "TestBotServoTuneOp")
public class TestBotServoTuneOp extends AbstractServoTuneOp {
    @Override
    protected RobotCfg createRobotCfg() {
        //create a new robot config and return it.
        //the superclass will extract the servos and do the rest.
        return new TestBotCfg(hardwareMap);
    }
}
