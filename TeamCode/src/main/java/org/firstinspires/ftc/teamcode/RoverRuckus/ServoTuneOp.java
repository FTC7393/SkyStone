package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import evlib.hardware.config.RobotCfg;
import evlib.opmodes.AbstractServoTuneOp;

/**
 * Created by ftc7393 on 12/3/2017.
 */
@TeleOp(name = "RoverRuckusServoTuneOp")

public class ServoTuneOp extends AbstractServoTuneOp {

    @Override
    protected RobotCfg createRobotCfg() {
        return new RoverRuckusRobotCfg(hardwareMap);
    }



}
