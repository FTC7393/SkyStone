package org.firstinspires.ftc.teamcode.SkyStone_Season.Autonomous;

public class ProportionalOffsetCalculator {
    private final double startValue;//the min first value below which correction is not needed
    private final double endValue;//the max second value above which correction becomes constant
    private final double correctionAtEnd;//the constant correction

    public ProportionalOffsetCalculator(double startValue, double endValue, double correctionAtEnd) {
        this.startValue = startValue;
        this.endValue = endValue;
        this.correctionAtEnd = correctionAtEnd;
    }

    public double calculateOffset(double firstValue) {
        if(firstValue < startValue) {
            return  0;
        }else if(firstValue<endValue) {
            //correction is now proportional to how far we are from the start value
            double m = (correctionAtEnd /(endValue - startValue));
            return m*(firstValue-startValue);
        } else {
            //above this value, correction is constant
            return correctionAtEnd;
        }
    }
}
