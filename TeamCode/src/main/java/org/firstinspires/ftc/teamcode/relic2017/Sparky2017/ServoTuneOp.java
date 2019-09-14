package org.firstinspires.ftc.teamcode.relic2017.Sparky2017;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import evlib.hardware.config.RobotCfg;
import evlib.opmodes.AbstractServoTuneOp;

/**
 * Created by ftc7393 on 12/3/2017.
 */
@TeleOp(name = "ServoTuneOp")
@Disabled

public class ServoTuneOp extends AbstractServoTuneOp {

    @Override
    protected RobotCfg createRobotCfg() {
        return new RobotCfg2017(hardwareMap);
    }

}
