package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import ftc.evlib.hardware.config.RobotCfg;
import ftc.evlib.opmodes.AbstractServoTuneOp;

/**
 * Created by ftc7393 on 12/3/2017.
 */
@TeleOp(name = "RoverRuckusServoTuneOp")
@Disabled
public class ServoTuneOp extends AbstractServoTuneOp {

    @Override
    protected RobotCfg createRobotCfg() {
        return new RoverRuckusRobotCfg(hardwareMap);
    }



}
