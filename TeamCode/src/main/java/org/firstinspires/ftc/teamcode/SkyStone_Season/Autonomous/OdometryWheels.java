package org.firstinspires.ftc.teamcode.SkyStone_Season.Autonomous;

import java.util.Base64;

import ftc.electronvolts.util.InputExtractor;

public class OdometryWheels {

   private InputExtractor EncoderValue;
   private double PastEncoderValue = 0;
   private InputExtractor GyroValue;
   private double distanceTraveled = 0;


    public OdometryWheels(InputExtractor<Double> EncoderValue, InputExtractor<Double> GyroValue){
        this.EncoderValue = EncoderValue;
        this.GyroValue = GyroValue;
    }

 // Calculating the distance travelled
 public void act(){
   double CurrentEncoderValue;
   double ChangeInDistance;
   CurrentEncoderValue = (double)EncoderValue.getValue();
   ChangeInDistance = CurrentEncoderValue - PastEncoderValue;
   distanceTraveled = distanceTraveled + ChangeInDistance;
   PastEncoderValue = CurrentEncoderValue;
 }

 public void reset(){
        distanceTraveled = 0;
 }
 public double getDistanceTraveled() {
        return distanceTraveled;
 }

}
