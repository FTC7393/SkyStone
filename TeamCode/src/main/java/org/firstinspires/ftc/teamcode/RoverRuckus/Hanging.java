package org.firstinspires.ftc.teamcode.RoverRuckus;

import evlib.hardware.motors.Motor;
import evlib.hardware.sensors.DigitalSensor;
import evlib.hardware.servos.ServoControl;
import ftc.electronvolts.util.DigitalInputEdgeDetector;

//import evlib.hardware.motors.MotorEnc;

public class Hanging {
    private final Motor hanging;
    private final ServoControl latch;
    boolean down=false;
    boolean up=false;
    boolean noLimUp=false;
    boolean noLimDown=false;
    boolean latchLimitPressed=false;
    boolean unlatchLimitPressed=false;
    final DigitalInputEdgeDetector latchLimit,unlatchLimit;
    private double hangingEncoder=0;
    public Hanging(Motor hanging, ServoControl latch, DigitalSensor latchLimit, DigitalSensor unlatchLimit){
        this.hanging=hanging;
        this.latch=latch;
        this.latchLimit=new DigitalInputEdgeDetector(latchLimit);
        this.unlatchLimit=new DigitalInputEdgeDetector(unlatchLimit);
    }
    public void act() {
        latchLimit.update();
        unlatchLimit.update();
        //hangingEncoder=hanging.getEncoderPosition();
        latchLimitPressed=latchLimit.isPressed();
        unlatchLimitPressed=unlatchLimit.isPressed();

        if(up==true) {
            if (!latchLimit.isPressed()) {
                hanging.setPower(1);
            } else {
                hanging.setPower(0);
            }
        }
        else if(down==true) {
            if (!unlatchLimit.isPressed()) {
                hanging.setPower(-1);
            } else {
                hanging.setPower(0);
            }
        }
        else if(noLimUp==true){
            hanging.setPower(1);

        }
        else if(noLimDown==true){
            hanging.setPower(-1);

        }
        else{
            hanging.setPower(0);

        }


        hanging.update();
    }
    public void upHanging(){
        up=true;
        down=false;


    }
    public void downHanging(){
        down=true;
        up=false;
    }
    public void stopHanging(){
        up=false;
        down=false;
    }
    public void noLimUpHanging(){
        noLimUp=true;
        noLimDown=false;


    }
    public void noLimDownHanging(){
        noLimUp=false;
        noLimDown=true;
    }

    public void unlatch(){
        latch.goToPreset(RoverRuckusRobotCfg.LatchPresets.UNLATCH);
  }
    public void latch(){
      latch.goToPreset(RoverRuckusRobotCfg.LatchPresets.LATCH);
  }
    public boolean islatchLimitPressed(){
        return latchLimitPressed;

    }
    public boolean isUnlatchLimitPressed(){
        return unlatchLimitPressed;

    }




    }
