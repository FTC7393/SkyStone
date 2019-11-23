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

import android.util.Log;

import ftc.evlib.hardware.motors.MotorEnc;
import ftc.evlib.hardware.sensors.AnalogSensor;
import ftc.evlib.hardware.sensors.DigitalSensor;
import ftc.evlib.util.StepTimer;
import ftc.electronvolts.util.DigitalInputEdgeDetector;
import ftc.electronvolts.util.Function;
import ftc.electronvolts.util.Functions;
import ftc.electronvolts.util.PIDController;

public class LinearSlide {
    /**
     * Created by ftc7393 on 10/27/2018.
     */


    private MotorEnc extension;
    double a=1.5;
    Function expo= Functions.eBased(a);
    boolean limitSwitchCount= false;


    private final DigitalInputEdgeDetector extensionLimit;
    private PIDController extensionPID;

    private double extensionEncoder=0;


    int maxExtensionPosition;


    int minExtensionPosition=-10000;   // !!! Change this to -10000 when limit switch is reinstalled !!!
    double extensionSetPoint=0;
    double extensionPower=0;



    public LinearSlide(MotorEnc extension, PIDController extensionPID, DigitalSensor extensionLimit, int maxExtensionPosition) {
        this.extension = extension;
        this.extensionLimit= new DigitalInputEdgeDetector(extensionLimit);
        this.extensionPID= extensionPID;
        this.maxExtensionPosition = maxExtensionPosition;
    }

    //StepTimer t = new StepTimer("Arm", Log.VERBOSE);

    public void act() {
//            t.start();
//            t.step("update limit switches");

        extensionLimit.update();

//            t.step("update encoders");

        extensionEncoder = extension.getEncoderPosition();

//            t.step("update potentiometer");
//
//            t.step("rotation logic");
//        t.step("extension limit switch");


        if((limitSwitchCount == false && extensionLimit.isPressed()==true) || extensionLimit.justPressed()==true){
            limitSwitchCount= true;

            extension.resetEncoder();
            //minExtensionPosition=(int)extensionEncoder;
            minExtensionPosition=0;
        }

//        t.step("extension logic");

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

//        t.step("extension set power");
        //        rotation.setPower(rotationPower);
        extension.setPower(extensionPower);


//        t.step("arm updates");
//        torque=-1*extensionLength*mass*gravity*java.lang.Math.cos(angle);
//        extensionForce=-1*mass*gravity*java.lang.Math.sin(angle);
        extension.update();
//        t.stop();

    }




    public void controlExtension(double x) {
        extensionSetPoint=extensionSetPoint + 50*(expo.f(-x));
    }
    public void setExtension( double x) {
        extensionSetPoint=x;
    }
    public void stopExtension() {
        extensionSetPoint = extensionEncoder;
    }

    // special function which returns to tell autonomous when the slide has done moving, specifically returns a boolean
    public boolean autoUse(double extension){
        boolean done=false;
        extensionSetPoint=extension;
        if((Math.abs(extensionSetPoint-extensionEncoder)>5)) {
            done=false;
        }
        else{
            done=true;
        }


        return done;

    }

    public double getExtensionEncoder(){return extensionEncoder;}
    public double getMinExtensionValue(){return minExtensionPosition;}


    public double getExtensionPower(){return extensionPower;}

    public double getExtensionSetPoint(){return extensionSetPoint;}
    public double getExponential(){return a;}

    //    public boolean getExtensionLimitSwitch(){return extensionLimit.isPressed();}

    boolean getLimitSwitchCount(){ return limitSwitchCount; }
}




