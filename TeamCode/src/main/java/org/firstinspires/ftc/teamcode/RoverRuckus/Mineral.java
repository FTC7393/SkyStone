package org.firstinspires.ftc.teamcode.RoverRuckus;

//import android.support.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Mineral implements Comparable<Mineral> {
    private final int x, y, width, height;
    private final double radius;
    private final float confidence;
    private final boolean isGold;
    private final int index;
    private final boolean isInsideFilterBox;
    private final boolean heightOk;
    private final boolean widthOk;
    private GoldDetector.Detection type;


    public Mineral(){
        this.type= GoldDetector.Detection.NOTHING;
        this.confidence = 0;
        this.x = 0;
        this.y = 0;
        this.width = 0;
        this.height = 0;
        this.radius=0;
        index = 0;
        isInsideFilterBox = false;
        heightOk = false;
        widthOk = false;
        this.isGold=false;
    }


    public Mineral(int x, int y, int width, int height,double radius,boolean isGold, float confidence,
                   int index, boolean isInside, boolean hOk, boolean wOk) {
        this.type= GoldDetector.Detection.NOTHING;
        this.confidence = confidence;
        this.x = x;
        this.y = y;
        this.width = width;
        this.height = height;
        this.radius=radius;
        this.isGold=isGold;
        this.index = index;
        this.isInsideFilterBox = isInside;
        this.heightOk = hOk;
        this.widthOk = wOk;

    }


    public int getX() {
        return x;
    }

    public int getY() {
        return y;
    }

    public int getWidth() {
        return width;
    }

    public int getHeight() {
        return height;
    }

    public double getRadius(){return radius;}
    public boolean isGold(){return isGold;}


    public float getConfidence() {
        return confidence;
    }
    public void setType(GoldDetector.Detection type){this.type=type;}
    public GoldDetector.Detection getType(){return type;}
    // for regular logging
    public String toString(){
        return String.format("%s,%.3f,%d,%d,%d,%d,%.2f",this.type,this.confidence,this.x,this.y,this.width,this.height,this.radius);
    }
    // for gold detector logging
    public String toStringExtended(){
        return String.format("%s,%.3f,%d,%d,%d,%d,%d,%d,%d,%.2f",
                this.type,this.confidence,this.x,this.y,isInsideFilterBox?1:0,this.width,widthOk?1:0,this.height,heightOk?1:0,this.radius);
    };
    @Override
    public int compareTo(// @NonNull
                                     Mineral another) {
        if(this.radius<another.radius){
            return -1;
        }
        else if(this.radius>another.radius){
            return 1;
        }
        return 0;
    }

    public void showInTelem(Telemetry telemetry) {
        telemetry.addData("Mineral "+index+":", isGold ? "GOLD" : "SILVER");
        telemetry.addData("Conf:", confidence);
        telemetry.addData("X", x);
        telemetry.addData("Y", y);
        telemetry.addData("inside", isInsideFilterBox);
        telemetry.addData("hieghtOK?", heightOk + " " + height);
        telemetry.addData("widthOK?", widthOk + " " + width);

    }
}
