package org.firstinspires.ftc.teamcode.relic2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.relic2017.Mechanisms.VuMarkFinder;
import org.firstinspires.ftc.teamcode.relic2017.Sparky2017.RobotCfg2017;

import ftc.evlib.driverstation.Telem;
import ftc.evlib.opmodes.AbstractAutoOp;
import ftc.evlib.statemachine.EVStateMachineBuilder;
import ftc.evlib.statemachine.EVStates;
//import ftc.evlib.vision.framegrabber.VuforiaFrameFeeder;
//import ftc.evlib.vision.processors.Particles;
import ftc.electronvolts.statemachine.EndConditions;
import ftc.electronvolts.statemachine.StateMachine;
import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.util.ResultReceiver;
import ftc.electronvolts.util.TeamColor;
import ftc.electronvolts.util.files.Logger;
import ftc.electronvolts.util.units.Angle;
import ftc.electronvolts.util.units.Time;

/**
 * This file was made by the electronVolts, FTC team 7393
 * Date Created: 12/14/16
 */
//@Autonomous(name = "ParticleScanOp")
//@Disabled
public class ParticleScanOp { // extends AbstractAutoOp<RobotCfg2017> {

//    private static final int FRAME_SIZE = 16;
//
//    private static final int FRAME_WIDTH = 9 * FRAME_SIZE;
//    private static final int FRAME_HEIGHT = 16 * FRAME_SIZE;
//
//
//    private enum S implements StateName {
//        WAIT_FOR_VUFORIA_INIT,
//        FIND_PARTICLES1,
//        SERVO1,
//        FIND_PARTICLES2,
//        DISPLAY,
//        TIMEOUT
//    }
//
//    private ResultReceiver<VuforiaFrameFeeder> vuforiaReceiver;
//    private Particles particles = new Particles();
//
//    private void addFindParticles(EVStateMachineBuilder b, StateName stateName, StateName nextState) {
//        b.add(stateName, EVStates.findParticlesState(nextState, S.TIMEOUT, Time.fromSeconds(2), particles, true));
//    }
//
//    @Override
//    public StateMachine buildStates() {
//
//        vuforiaReceiver = VuforiaFrameFeeder.initInNewThread(VuMarkFinder.VuforiaKey, R.id.cameraMonitorViewId, FRAME_WIDTH, FRAME_HEIGHT);
//
//        EVStateMachineBuilder b = robotCfg.createEVStateMachineBuilder(S.WAIT_FOR_VUFORIA_INIT, TeamColor.RED, Angle.fromDegrees(2));
//
//        b.add(S.WAIT_FOR_VUFORIA_INIT, EVStates.empty(b.t(S.FIND_PARTICLES1, EndConditions.receiverReady(vuforiaReceiver))));
//        addFindParticles(b, S.FIND_PARTICLES1, S.FIND_PARTICLES2);
////        b.addServo(S.SERVO1, S.FIND_PARTICLES2, RobotCfg2017.MainServoName.PHONE, RobotCfg2017.PhonePresets.FRONT, 0.5, true);
//        addFindParticles(b, S.FIND_PARTICLES2, S.DISPLAY);
//        b.add(S.DISPLAY, EVStates.displayParticles(particles));
//        b.add(S.TIMEOUT, EVStates.displayParticles(particles));
//
//        return b.build();
//    }
//
//    @Override
//    protected RobotCfg2017 createRobotCfg() {
//        return new RobotCfg2017(hardwareMap);
//    }
//
//    @Override
//    protected Logger createLogger() {
//        return null;
//    }
//
//    @Override
//    protected void setup_act() {
//        Telem.displayVuforiaReadiness(vuforiaReceiver);
//    }
//
//    @Override
//    protected void go() {
//
//    }
//
//    @Override
//    protected void act() {
//        telemetry.addData("state", stateMachine.getCurrentStateName());
//        Telem.displayVuforiaReadiness(vuforiaReceiver);
//    }
//
//    @Override
//    protected void end() {
//
//    }
}
