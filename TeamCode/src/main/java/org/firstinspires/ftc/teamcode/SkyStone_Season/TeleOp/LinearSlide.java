package org.firstinspires.ftc.teamcode.SkyStone_Season.TeleOp;

import android.util.Log;

import java.util.List;

import ftc.electronvolts.util.DigitalInputEdgeDetector;
import ftc.electronvolts.util.Function;
import ftc.electronvolts.util.Functions;
import ftc.electronvolts.util.PIDController;
import ftc.evlib.hardware.motors.MotorEnc;
import ftc.evlib.hardware.sensors.AnalogSensor;
import ftc.evlib.hardware.sensors.DigitalSensor;
import ftc.evlib.util.StepTimer;

public class LinearSlide {
  import android.util.Log;

import ftc.evlib.hardware.motors.MotorEnc;
import ftc.evlib.hardware.sensors.AnalogSensor;
import ftc.evlib.hardware.sensors.DigitalSensor;
import ftc.evlib.util.StepTimer;
import ftc.electronvolts.util.DigitalInputEdgeDetector;
import ftc.electronvolts.util.Function;
import ftc.electronvolts.util.Functions;
import ftc.electronvolts.util.PIDController;

    /**
     * Created by ftc7393 on 10/27/2018.
     */

    public class Arm {
        double extensionLength;
        double torque;
        double extensionForce;
        private MotorEnc extension;
        double a=1.5;
        Function expo= Functions.eBased(a);
        boolean isLocked=false;
        int limitSwitchCount=0;


        private final DigitalInputEdgeDetector extensionLimit;
        private PIDController extensionPID;

        private double extensionEncoder=0;

        // Presets:
        // Driver2 dpad UP: COLLECT ARM, Extend 1985
        // Driver2 dpad DOWN: STOW ARM, Extend 0
        // Driver2 left_bumper: SILVER DUMP, Extend 1596, Rotate 2.33
        // Driver2 right_bumper: GOLD DUMP, Extend 1985, Rotate 2.47



        int dumpSilverExtensionPosition = 1596;
        int dumpGoldExtensionPosition = 1985+204-100;
        int collectExtensionPosition = 1985;
        int stowExtensionPosition = 0;

        int maxExtensionPosition=2685;


        int minExtensionPosition=-10000;   // !!! Change this to -10000 when limit switch is reinstalled !!!
        double extensionSetPoint=0;
        double extensionPower=0;



        public LinearSlide(MotorEnc extension, DigitalSensor extensionLimit) {
            this.extension = extension;
            this.extensionLimit= new DigitalInputEdgeDetector(extensionLimit);
            this.rotation = rotation;

            this.rotationPID=new PIDController(0.001,0.0002,.00006,1);
            this.extensionPID=new PIDController(0.003,0,0,1);
            this.potentiometer=potentiometer;

        }

        StepTimer t = new StepTimer("Arm", Log.VERBOSE);

        public void act() {
            t.start();
            t.step("update limit switches");

            extensionLimit.update();

            t.step("update encoders");

            rotationEncoder=rotation.getEncoderPosition();
            extensionEncoder=extension.getEncoderPosition();

            t.step("update potentiometer");
            potentiometerValue=potentiometer.getValue();

            //if(isLocked==true){
            //    rotationSetPoint=rotationLockSetPoint;  // Always control to the point we were at when we pressed the lock button
            //rotation.setPower(0);
            //}
            //else{
//            rotationPower=rotationPID.computeCorrection(rotationSetPoint,rotationEncoder);
            //   rotationPower=1;

            t.step("rotation logic");

            // Check if we are controlling to a preset
            if(rotationPotentiometerControl) {
                if(Math.abs(rotationPotentiometerTarget-potentiometerValue) > 0.05) {
                    controlRotationAuto(-1 * (rotationPotentiometerTarget - potentiometerValue));
                } else {
                    stopRotation();
                }
            }

            // Check for the absolute rotation limits
            if( potentiometerValue >= potentiometerMaxPosition &&
                    previousPotentiometerValue < potentiometerMaxPosition){
                rotationSetPoint = (int)rotationEncoder;
                maxRotationPosition = (int)rotationEncoder;

            } else if(potentiometerValue <= potentiometerMinPosition  &&
                    previousPotentiometerValue > potentiometerMinPosition ) {
                rotationSetPoint = (int) rotationEncoder;
                minRotationPosition = (int) rotationEncoder;
            }

            if(rotationSetPoint > maxRotationPosition){
                rotationSetPoint = maxRotationPosition;
            } else if(rotationSetPoint < minRotationPosition) {
                rotationSetPoint = minRotationPosition;
            }
            //}
            previousPotentiometerValue=potentiometerValue;

            t.step("set rotation");
            rotation.setPosition((int)rotationSetPoint,1);
            //rotation.setSpeed(rotationSetSpeed);


//        if( rotationLimit.justPressed()==true){
//            rotation.resetEncoder();
//
//        }
            t.step("extension limit switch");
            if((limitSwitchCount == 0 && extensionLimit.isPressed()==true) || extensionLimit.justPressed()==true){
                limitSwitchCount=1;

                extension.resetEncoder();
                //minExtensionPosition=(int)extensionEncoder;
                minExtensionPosition=0;
            }

            t.step("extension logic");

//        rotation.setSpeed(rotationPower);

//        if(rotationSetPoint<minRotationPosition){
//            rotationSetPoint=minRotationPosition;
//        }
            if(extensionSetPoint>maxExtensionPosition){
                extensionSetPoint=maxExtensionPosition;
            }
            if(extensionSetPoint<minExtensionPosition){
                extensionSetPoint=minExtensionPosition;
            }
            extensionPower=extensionPID.computeCorrection(extensionSetPoint,extensionEncoder);

            t.step("extension set power");
            //        rotation.setPower(rotationPower);
            extension.setPower(extensionPower);


            t.step("arm updates");
//        torque=-1*extensionLength*mass*gravity*java.lang.Math.cos(angle);
//        extensionForce=-1*mass*gravity*java.lang.Math.sin(angle);
            extension.update();
            rotation.update();
            t.stop();

        }

        public void zeroRotation(){rotationSetPoint=minRotationPosition;}
        public void endRotation(){rotationSetPoint=maxRotationPosition;}
        public void dumpSilverRotation(){
            rotationPotentiometerTarget = rotationSilverPosition;
            rotationPotentiometerControl = true;
        }
        public void dumpGoldRotation(){
            rotationPotentiometerTarget = rotationGoldPosition;
            rotationPotentiometerControl = true;
        }

        public void controlRotation(double y){
            if(potentiometerValue > rotationFastZoneMin && potentiometerValue < rotationFastZoneMax) {
                rotationSetPoint = rotationSetPoint + 80 * (expo.f(-y));
            } else {
                rotationSetPoint = rotationSetPoint + 50*(expo.f(-y));
            }
            rotationPotentiometerControl = false;
        }

        public void controlRotationAuto(double y) {
            if(potentiometerValue > rotationFastZoneMin && potentiometerValue < rotationFastZoneMax) {
                rotationSetPoint = rotationSetPoint + 80*(-y);
            } else {
                rotationSetPoint = rotationSetPoint + 76*(-y);
            }

        }

        public void stopRotation() {
            rotationSetPoint = rotationEncoder;
            rotationPotentiometerControl = false;
        }

        //public void controlRotation(double y) {
        //    rotationSetPoint = rotation.getEncoderPosition() + 120*(expo.f(-y));
        //}
        //public void controlRotation(double y){rotationSetSpeed=0.25*(expo.f(-y));}


        //    public void controlRotation(double y){rotation.setPower(y);}
//    public void controlExtension(double x){extension.setPower(x);}
        public void controlExtension(double x) {
            extensionSetPoint=extensionSetPoint + 50*(expo.f(-x));
        }
        public void stopExtension() {
            extensionSetPoint = extensionEncoder;
        }

        public void zeroExtension(){extensionSetPoint=minExtensionPosition;}
        public void endExtension(){extensionSetPoint=maxExtensionPosition;}
        public void dumpGoldExtension(){extensionSetPoint=dumpGoldExtensionPosition;}
        public void dumpSilverExtension(){extensionSetPoint=dumpSilverExtensionPosition;}
        public void collectExtension(){extensionSetPoint=collectExtensionPosition;}
        public void stowExtension(){extensionSetPoint=stowExtensionPosition;}
        public boolean autoUse(double extension,double rotation){
            boolean done=false;
            extensionSetPoint=extension;
            rotationPotentiometerTarget = rotation;
            rotationPotentiometerControl = true;
            if((Math.abs(rotationPotentiometerTarget-potentiometerValue) > 0.05)||(Math.abs(extensionSetPoint-extensionEncoder)>5)) {
                done=false;
            }
            else{
                done=true;
            }


            return done;

        }
        //    public boolean autoCollect(){
//        boolean done=false;
//        extensionSetPoint=collectExtensionPosition;
//        rotationPotentiometerTarget = coll;
//        rotationPotentiometerControl = true;
//        if((Math.abs(rotationPotentiometerTarget-potentiometerValue) > 0.05)||(Math.abs(extensionSetPoint-extensionEncoder)>5)) {
//            done=false;
//        }
//        else{
//            done=true;
//        }
//
//
//        return done;
//
//    }
        public double getRotationEncoder(){return rotationEncoder;}
        public double getExtensionEncoder(){return extensionEncoder;}
        public double getMinRotationValue(){return minRotationPosition;}
        public double getMinExtensionValue(){return minExtensionPosition;}


        public double getRotationPower(){return rotationPower;}
        public double getExtensionPower(){return extensionPower;}

        public double getRotationSetPoint(){return rotationSetPoint;}
        public double getExtensionSetPoint(){return extensionSetPoint;}
        public double getExponential(){return a;}

        public MotorEnc getRotationMotor() { return rotation; }
        public double getPotentiometerValue(){return potentiometerValue;}
        //    public boolean getExtensionLimitSwitch(){return extensionLimit.isPressed();}
//    public void lockRotation(){
//        isLocked=true;
//        rotationLock.goToPreset(RoverRuckusRobotCfg.rotationLockPresets.LOCKED);
//        rotationLockSetPoint = rotationEncoder;  // Save the point we were at when we pressed the lock button62
//    }
//    public void unlockRotation(){isLocked=false;rotationLock.goToPreset(RoverRuckusRobotCfg.rotationLockPresets.UNLOCKED);}
        double theta1;
        double theta2;
        double r1;
        double r2;

        double armAjust (double r){
            double slope,b,theta;
            slope=(theta2-theta1)/(r2-r1);
            b=-((slope*r1)-theta1);
            theta=slope*r+b;
            return theta;
        }
        int getLimitSwitchCount(){
            return limitSwitchCount;

        }



    }




}