package org.firstinspires.ftc.teamcode.futurefest2019;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import ftc.evlib.hardware.config.RobotCfg;
import ftc.evlib.opmodes.AbstractServoTuneOp;

@TeleOp(name = "FutureFestServoTuneOp")
@Disabled
public class FutureFestServoTuneOp extends AbstractServoTuneOp {

    protected RobotCfg createRobotCfg() {
        //create a new SampleRobotConfig and return it.
        //the superclass will extract the servos and do the rest.
        return new FutureFestRobotCfg(hardwareMap);
    }
}
