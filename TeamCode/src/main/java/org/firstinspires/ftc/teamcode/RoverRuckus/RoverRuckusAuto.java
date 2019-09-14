package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.qualcomm.robotcore.util.ElapsedTime;

import evlib.hardware.control.MecanumControl;
import evlib.hardware.control.RotationControl;
import evlib.hardware.control.RotationControls;
import evlib.hardware.control.TranslationControl;
import evlib.hardware.control.TranslationControls;
import evlib.hardware.motors.MecanumMotors;
import evlib.hardware.sensors.Gyro;
import evlib.opmodes.AbstractOp;
import ftc.electronvolts.util.Vector2D;
import ftc.electronvolts.util.files.Logger;
import ftc.electronvolts.util.units.Angle;
import ftc.electronvolts.util.units.Distance;
import ftc.electronvolts.util.units.Time;

/**
 * Created by ftc7393 on 10/13/2018.
 */
//@Autonomous(name = "RoverRuckusAuto")

public class RoverRuckusAuto extends AbstractOp<RoverRuckusRobotCfg> {
    private StateName state = StateName.ANTITIP;

    private ElapsedTime runtime = new ElapsedTime();

    /// TOM FOUND THE PROBLEM:  robotCfg is NULL when the variable initializers are called--they happen before the AbstractOp creat
    Gyro gyro;
    MecanumControl mecanumControl;
    Angle tolerance=Angle.fromRadians(0);
    double maxAngularSpeed=.5;


    private enum StateName {
        LOCK_ROTATION,
        MOVE_OFF_HOOK,
        START,
        RELEASE_MARKER,
        ANTITIP,

        DETECT_GOLD,

        GOLD_ALIGN, DRIVE_TO_DEPOT, DRIVE_LITTLE, RELEASE_LATCH

    }

    @Override
    protected RoverRuckusRobotCfg createRobotCfg() {
        return new RoverRuckusRobotCfg(hardwareMap);
    }

    @Override
    protected Logger createLogger() {
        return null;
    }

    @Override
    protected void setup() {
        gyro = robotCfg.getGyro();
        mecanumControl = robotCfg.getMecanumControl();
    }

    @Override
    protected void setup_act() {

    }

    @Override
    protected void go() {

    }

    @Override
    protected void pre_act() {


    }

    @Override
    protected void act() {
        Distance distanceTravelled;
        long lastTime = 0;

        telemetry.addData("state",state.name());
        boolean isDone=false;
        telemetry.addData("isDone?",isDone);


        int redLeftColor = 0;//robotCfg.getLeftColorSensor().red();
        int blueLeftColor = 0;//robotCfg.getLeftColorSensor().blue();
        int redRightColor = 0;//robotCfg.getRightColorSensor().red();
        int blueRightColor = 0;//robotCfg.getRightColorSensor().blue();
        telemetry.addData("blue right color",blueRightColor);
        telemetry.addData("blue left color",blueLeftColor);
        telemetry.addData("red right color",redRightColor);
        telemetry.addData("red left color",redRightColor);
//        if(state==StateName.START){
//            distanceTravelled = Distance.zero();
//            lastTime = System.currentTimeMillis();
////                timeEC.init();
////                gyroEC.init();
//            mecanumControl.setControl(rotationControl, translationControl);
//            state=StateName.DRIVE_LITTLE;
//
//        }
        if(state== StateName.ANTITIP){
            robotCfg.getBackFoot().goToPreset(RoverRuckusRobotCfg.BackFootPresets.DEPLOYED);
            runtime.reset();

            state= StateName.DRIVE_LITTLE;

        }
        if (state == StateName.DRIVE_LITTLE) {

            if (runtime.seconds() < 10) {
                robotCfg.backLeft.setPower(1);
                robotCfg.backRight.setPower(1);
                robotCfg.frontLeft.setPower(1);
                robotCfg.frontRight.setPower(1);


            }
            robotCfg.backLeft.setPower(0);
            robotCfg.backRight.setPower(0);
            robotCfg.frontLeft.setPower(0);
            robotCfg.frontRight.setPower(0);
            state= StateName.RELEASE_MARKER;

        }

//        if (state == StateName.DRIVE_TO_DEPOT) {
//
//            telemetry.addData("blue right color",blueRightColor);
//            telemetry.addData("blue left color",blueLeftColor);
//            telemetry.addData("red right color",redRightColor);
//            telemetry.addData("red left color",redRightColor);
//
//
//                mecanumDrive(.5,180,0);
//                if(blueRightColor >  &&blueLeftColor<2 && redRightColor < 2&&redLeftColor<2){
//
//                }
//
//
//
//
//            mecanumControl.stop();
//
//
//
//
//        }
       // if(state==StateName.RELEASE_MARKER){
         //   robotCfg.getCollector().leftDoor();
        //}
//        final ResultReceiver<SamplingOrderDetector.GoldLocation> goldLocationReceiver= SamplingOrderExample.initThread(hardwareMap);
//
//        SamplingOrderDetector.GoldLocation goldLocation;
//        if(state==StateName.DETECT_GOLD){
//            while(SamplingOrderDetector.GoldLocation.UNKNOWN==goldLocationReceiver.getValue()) {
//                goldLocation = goldLocationReceiver.getValue();
//            }
//            state=StateName.GOLD_ALIGN;
//
//
//
//        }
//
////        if(state==StateName.GOLD_ALIGN) {
////            while (GoldAlignExample.getXpos() < 250) {
////
////
////            }
////            while (GoldAlignExample.getXpos() > 350) {
////
////
////            }
////
////        }


    }

    @Override
    protected void post_act() {

    }

    @Override
    protected void end() {

    }

    @Override
    protected Time getMatchTime() {
        return null;
    }

    public void mecanumDrive( final double velocity, final double direction,final double orientation) {

        mecanumControl.setDriveMode(MecanumMotors.MecanumDriveMode.NORMALIZED);

        RotationControl rotationControl=RotationControls.gyro(gyro, Angle.fromDegrees(orientation), tolerance, maxAngularSpeed);
        TranslationControl translationControl=TranslationControls.constant(velocity,Angle.fromDegrees(direction));


        mecanumControl.setControl(rotationControl,translationControl);
    }
    public  boolean mecanumDrive( final Distance distance, final double velocity, final double direction, final double orientation)
    {
        return mecanumDrive(distance, mecanumControl, RotationControls.gyro(gyro, Angle.fromDegrees(orientation), tolerance, maxAngularSpeed), TranslationControls.constant(velocity, Angle.fromDegrees(direction)));
    }
    public  boolean mecanumDrive(final Distance distance,  MecanumControl mecanumControl, final RotationControl rotationControl, final TranslationControl translationControl)
    {
        mecanumControl.setDriveMode(MecanumMotors.MecanumDriveMode.NORMALIZED);
//        double speedMetersPerMillisecond = mecanumControl.getMaxRobotSpeed(Angle.subtract(direction, orientation)).metersPerMillisecond() * velocity;
//        double durationMillis = Math.abs(distance.meters() / speedMetersPerMillisecond);
//        final EndCondition gyroEC = EVEndConditions.gyroCloseTo(gyro, orientation, tolerance);
//        final EndCondition timeEC = EVEndConditions.timed((long) durationMillis);


            Distance distanceTravelled = Distance.zero();
            long lastTime = 0;

                distanceTravelled = Distance.zero();
                lastTime = System.currentTimeMillis();
//                timeEC.init();
//                gyroEC.init();
                mecanumControl.setControl(rotationControl, translationControl);


                long now = System.currentTimeMillis();
                Time deltaTime = Time.fromMilliseconds(lastTime - now);
                lastTime = now;

//                Log.i("Drive", "distanceTravelled: " + distanceTravelled);
//                Log.i("Drive", "delta distance: " + Distance.multiply(
//                        mecanumControl.getMaxRobotSpeed(Angle.subtract(direction, Angle.fromDegrees(gyro.getHeading()))).getDistance(deltaTime),
//                        velocity * mecanumControl.getMecanumMotors().getScaleFactor()
//                ));
//                Log.i("Drive", "maxRobotSpeed: " + mecanumControl.getMaxRobotSpeed(Angle.subtract(direction, Angle.fromDegrees(gyro.getHeading()))));
//                Log.i("Drive", "velocity * scaleFactor: " + velocity * mecanumControl.getMecanumMotors().getScaleFactor());
//                Log.i("Drive", "drive direction: " + Angle.subtract(direction, Angle.fromDegrees(gyro.getHeading())));
//
                Vector2D translation = new Vector2D(
                        mecanumControl.getVelocityX(),
                        mecanumControl.getVelocityY()
                );

                distanceTravelled = Distance.add(
                        distanceTravelled,
                        Distance.multiply(
                                mecanumControl.getMaxRobotSpeed(translation.getDirection()).getDistance(deltaTime),
//                                mecanumControl.getMaxRobotSpeed(Angle.subtract(translation.getDirection(), Angle.fromDegrees(gyro.getHeading()))).getDistance(deltaTime),
                                translation.getLength() * mecanumControl.getMecanumMotors().getScaleFactor()
                        ).abs()
                );
                return distanceTravelled.meters() >= distance.meters();
//                return timeEC.isDone();
//                if (timeEC.isDone()) {
//                    mecanumControl.setTranslationControl(TranslationControls.ZERO);
//                    return gyroEC.isDone();
//                }
//                return false;



        };
    }

