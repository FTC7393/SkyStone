package org.firstinspires.ftc.teamcode.SkyStone_Season.TeleOp;

import ftc.electronvolts.util.DigitalInputEdgeDetector;
import ftc.electronvolts.util.Function;
import ftc.electronvolts.util.Functions;
import ftc.electronvolts.util.PIDController;
import ftc.evlib.hardware.motors.MotorEnc;
import ftc.evlib.hardware.sensors.DigitalSensor;

public class LinearSlide {
    /**
     * Created by ftc7393 on 10/27/2018.
     */


    private MotorEnc extension;
    double a=1.5;
    Function expo= Functions.eBased(a);
    boolean lowerLimitResetComplete = false;


    private final DigitalInputEdgeDetector lowerLimit;
    private final DigitalInputEdgeDetector upperLimit;
    private PIDController extensionPID;

    private double extensionEncoder=0;

    private final int tolerance;

    int maxExtensionPosition;
    


    int minExtensionPosition=-10000;   // !!! Change this to -10000 when limit switch is reinstalled !!!
    double extensionSetPoint=0;
    double extensionPower=0;



    public LinearSlide(MotorEnc extension, PIDController extensionPID, int maxExtensionPosition, int tolerance ) {
        this.extension = extension;
        this.lowerLimit= null;
        this.lowerLimitResetComplete= true;
        this.extensionPID= extensionPID;
        this.maxExtensionPosition = maxExtensionPosition;
        this.minExtensionPosition = 0;
        this.upperLimit = null;
        this.tolerance = tolerance;
    }

    public LinearSlide(MotorEnc extension, PIDController extensionPID, int maxExtensionPosition, int tolerance,
                       DigitalSensor lowerLimit) {
        this.extension = extension;
        this.lowerLimit= new DigitalInputEdgeDetector(lowerLimit);
        this.extensionPID= extensionPID;
        this.maxExtensionPosition = maxExtensionPosition;
        this.minExtensionPosition = -10000;
        this.upperLimit = null;
        this.tolerance = tolerance;
    }

    public LinearSlide(MotorEnc extension, PIDController extensionPID, int maxExtensionPosition, int tolerance,
                       DigitalSensor lowerLimit, DigitalSensor upperLimit) {
        this.extension = extension;
        this.lowerLimit= new DigitalInputEdgeDetector(lowerLimit);
        this.extensionPID= extensionPID;
        this.maxExtensionPosition = maxExtensionPosition;
        this.minExtensionPosition = -10000;
        this.upperLimit = new DigitalInputEdgeDetector(upperLimit);
        this.tolerance = tolerance;
        
    }

    

    //StepTimer t = new StepTimer("Arm", Log.VERBOSE);

    public void pre_act() {
        if (lowerLimit != null) {
            lowerLimit.update();
        }

        if (upperLimit != null) {
            upperLimit.update();
        }

//            t.step("update encoders");

        extensionEncoder = extension.getEncoderPosition();

    }

    public void act() {
//            t.start();
//            t.step("update limit switches");
        

//            t.step("update potentiometer");
//
//            t.step("rotation logic");
//        t.step("extension limit switch");


        if (lowerLimit != null) {
            if ((!lowerLimitResetComplete && lowerLimit.isPressed()) || lowerLimit.justPressed()) {
                lowerLimitResetComplete = true;

                extension.resetEncoder();
                minExtensionPosition = 0;
            }
        }

        if (upperLimit != null) {
            if (upperLimit.isPressed() && extensionSetPoint >= extensionEncoder){
                extensionSetPoint = extensionEncoder;
            }
        }

        if(extensionSetPoint>maxExtensionPosition){
            extensionSetPoint=maxExtensionPosition;
        }
        if(extensionSetPoint<minExtensionPosition){
            extensionSetPoint=minExtensionPosition;
        }
        extensionPower=extensionPID.computeCorrection(extensionSetPoint,extensionEncoder);

//        t.step("extension set power");

        extension.setPower(extensionPower);


//        t.step("arm updates");
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
        extensionSetPoint=extension;
        return isDone();
    }

    public boolean isDone(){
      return (Math.abs(extensionSetPoint-extensionEncoder)<= tolerance);
    }

    public double getExtensionEncoder(){return extensionEncoder;}
    public int getMinExtensionValue(){return minExtensionPosition;}


    public double getExtensionPower(){return extensionPower;}

    public double getExtensionSetPoint(){return extensionSetPoint;}
    public double getExponential(){return a;}

    //    public boolean getExtensionLimitSwitch(){return extensionLimit.isPressed();}

    boolean getLowerLimitResetComplete(){ return lowerLimitResetComplete; }

    public boolean getUpperLimit(){
        if (upperLimit != null) {
            return upperLimit.isPressed();
        }
        return false;
    }

    public boolean getLowerLimit(){
        if (lowerLimit != null) {
            return lowerLimit.isPressed();
        }
        return false;
    }
}




