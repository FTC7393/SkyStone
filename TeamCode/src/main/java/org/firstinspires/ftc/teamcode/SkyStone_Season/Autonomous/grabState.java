package org.firstinspires.ftc.teamcode.SkyStone_Season.Autonomous;

import org.firstinspires.ftc.teamcode.SkyStone_Season.SkystoneRobotCfg;
import org.firstinspires.ftc.teamcode.SkyStone_Season.TeleOp.FlyWheels;

import ftc.electronvolts.statemachine.State;
import ftc.electronvolts.statemachine.StateName;

public class grabState implements State {
    FlyWheels flywheels;
    SkystoneRobotCfg robotCfg;
    boolean isFirst = true;
    @Override
    public StateName act() {
        flywheels = robotCfg.getFlyWheels();
        if (isFirst) {
            isFirst = false;

        }
        return null;
    }
}
