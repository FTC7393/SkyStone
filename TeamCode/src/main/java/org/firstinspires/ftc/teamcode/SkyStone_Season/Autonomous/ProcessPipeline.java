package org.firstinspires.ftc.teamcode.SkyStone_Season.Autonomous;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.HashMap;
import java.util.Map;

import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.util.BasicResultReceiver;
import ftc.electronvolts.util.InputExtractor;


//enum StoneType {
//    SKY, REG;
//}
//
//class SkystoneDetector {
//    private final double minBlueForRegStone;
//    private final int numContinousDetectionsRequired;
//
//    private int numContinuousStoneDetections = 0;
//    private int numContinuousSkystoneDetections = 0;
//
//    private StoneType stype = null;
//
//    public SkystoneDetector(double minBlueForSkystone, int numContinousDetectionsRequired) {
//        this.minBlueForRegStone = minBlueForSkystone;
//        this.numContinousDetectionsRequired = numContinousDetectionsRequired;
//    }
//
//
//    StoneType getStoneType(double blueValue) {
//        if (stype == null) {
//            if (blueValue > minBlueForRegStone) {
//                numContinuousSkystoneDetections = 0;
//                numContinuousStoneDetections++;
//            } else {
//                numContinuousSkystoneDetections++;
//                numContinuousStoneDetections = 0;
//            }
//
//            if(numContinuousSkystoneDetections > numContinousDetectionsRequired) {
//                stype = StoneType.SKY;
//            } else if (numContinuousStoneDetections > numContinousDetectionsRequired) {
//                stype = StoneType.REG;
//            }
//        }
//
//        return stype;
//    }
//}

//class ColorDetector {
//    private final double maxVariance;
//    private final int minSimilarValues;
//    private int similarCount = 0;
//    private double runningAverage = -1;
//
//    public ColorDetector(double maxVariance, int minSimilarValues) {
//        this.maxVariance = maxVariance;
//        this.minSimilarValues = minSimilarValues;
//    }
//
//    public Double getColor(double thisBlueLevel) {
//        if (similarCount > minSimilarValues) {
//            return runningAverage;
//        }
//
//        if (runningAverage == -1) {
//            runningAverage = thisBlueLevel;
//            similarCount = 1;
//        } else {
//            if (Math.abs(thisBlueLevel - runningAverage) < maxVariance) {
//                runningAverage = (runningAverage * similarCount + thisBlueLevel) / ++similarCount;
//            } else {
//                runningAverage = thisBlueLevel;
//                similarCount = 1;
//            }
//        }
//        if (similarCount > minSimilarValues) {
//            return runningAverage;
//        } else {
//            return null;
//        }
//    }
//
//    public double getRunningAverage() {
//        return runningAverage;
//    }
//}


class ProcessPipeline extends OpenCvPipeline {

    private final BasicResultReceiver<StateName> foundSkystoneRR;
    private final int minNumContinousForPositiveId;
    private final int numSettleCycles;
//    private final double minBlueForRegStone;
    private final StateName [] skystoneStates;
    private final int maxTries;
    private final StateName defaultState;

    private int processCounter = 0;

    private double blueDiff = 0;
    private double ac = 0;
    private double blue1 = -1;
    private double blue2;
    private String ratio = "unset";
    public final InputExtractor<String> keyII = new InputExtractor<String>() {
        @Override
        public String getValue() {
            return ratio;
        }
    };
    public final InputExtractor<Double> blue1II = new InputExtractor<Double>() {
        @Override
        public Double getValue() {
            return blue1;
        }
    };
    public final InputExtractor<Double> blue2II = new InputExtractor<Double>() {
        @Override
        public Double getValue() {
            return blue2;
        }
    };

    private final InputExtractor<Double> AvgColorII = new InputExtractor<Double>() {
        @Override
        public Double getValue() {
            return ac;
        }
    };
    private final InputExtractor<Double> blueDiffII = new InputExtractor<Double>() {
        @Override
        public Double getValue() {
            return blueDiff;
        }
    };

    public static String threadName = "";
    int x1 = 200, y1 = 350, w1 = 20, h1 = 50;
    int x2 = x1,  y2 = 260, w2 = 20, h2 = 50;
    Mat m1;
    Mat m2;

    public ProcessPipeline(BasicResultReceiver<StateName> foundSkystoneRR, int minNumContinousForPositiveId, int numSettleCycles, double minBlueForRegStone, StateName[] skystoneStates, int maxTries, StateName defaultState) {
        this.foundSkystoneRR = foundSkystoneRR;
        this.minNumContinousForPositiveId = minNumContinousForPositiveId;
        this.numSettleCycles = numSettleCycles;
//        this.minBlueForRegStone = minBlueForRegStone;
        this.skystoneStates = skystoneStates;
        this.maxTries = maxTries;
        this.defaultState = defaultState;
//        stone1 = new SkystoneDetector(minBlueForRegStone,minNumContinousForPositiveId);
//        stone2 = new SkystoneDetector(minBlueForRegStone,minNumContinousForPositiveId);
    }

    @Override
    public Mat processFrame(Mat input) {
        if(threadName.length() == 0) {
            //only set threadName during first time through processFrame
            threadName = Thread.currentThread().getName();
        }

        Rect rect1 = new Rect(x1, y1, w1, h1);
        Rect rect2 = new Rect(x2, y2, w2, h2);
        if (processCounter++ >=numSettleCycles) {

            if (foundSkystoneRR.isReady()) {
//                blue1 = -3;
                // do nothing here...
            } else if (processCounter > maxTries) {
                foundSkystoneRR.setValue(defaultState);
                blue1 = -5;
            } else {

                blue1 = getLowestAvgBlue(input, rect1);
                blue2 = getLowestAvgBlue(input, rect2);

                StateName nextState = getAnswer(blue1, blue2);
                foundSkystoneRR.setValue(nextState);


//                StoneType t1 = stone1.getStoneType(blue1);
//                StoneType t2 = stone2.getStoneType(blue2);
//                if ((t1 != null) && (t2 != null)) {
//                    StateName nextState = getAnswer(t1, t2);
//                    foundSkystoneRR.setValue(nextState);
//                }

//                Double c1 = stone1.getColor(blue1);
//                Double c2 = stone2.getColor(blue2);
//                if ((c1 != null) && (c2 != null)) {
//                    StateName nextState = getAnswer(c1, c2);
//                    foundSkystoneRR.setValue(nextState);
//                }
            }
        }
        Imgproc.rectangle(input, rect1, new Scalar(255, 0, 0), 5);
        Imgproc.rectangle(input, rect2, new Scalar(255, 0, 0), 5);


        return input;
    }

    private StateName getAnswer(double c1, double c2) {
        double blueRatio = c1/c2;
        ratio = String.format("%4.2f",blueRatio);
        if (blueRatio < 0.6) {
            // c1 (left of the two) has less blue, so it is a dark skystone;
            // c1 is the middle of the options (1 out of 0,1,2)
            return skystoneStates[1];
        }
        if (blueRatio > 1.8) {
            // c1 (left of the two) has more blue, so it is NOT the darker skystone;
            // c2 is the right of the options (2 out of 0,1,2)
            return skystoneStates[2];
        }
        // both have the nearly the same value, so they must both be regular stones;
        // so the skystone is on the left (option 0 of 0,1,2)
        return skystoneStates[0];
    }

//    private StateName getAnswer(StoneType t1, StoneType t2) {
//        Map<String, StateName> map = new HashMap<>();
//        map.put("REG REG", skystoneStates[0]); // left
//        map.put("SKY REG", skystoneStates[1]); // middle
//        map.put("REG SKY", skystoneStates[2]); // right
//        map.put("SKY SKY", skystoneStates[2]); // who knows- go with right
//
//        key = t1.name() + " " + t2.name();
//        return map.get(key);
//    }



    private double getAvgBlueInMiddle(Mat input, Rect rect) {
        m1 = new Mat(input, rect).clone();
        int nw = 5, nh = 5;
        m2 = new Mat(nw, nh, input.type());
        Size s = new Size(nw, nh);
        Imgproc.resize(m1, m2, s);
        double[] colors = m2.get(2, 2);
        double b = colors[0];
        double g = colors[1];
        double r = colors[2];

        double avgColor = Math.sqrt(b * 2 + g * 2 + r * 2);
        ac = avgColor;
        blueDiff = b;

        m1.release();
        m2.release();

        return b;
    }
    private double getLowestAvgBlue(Mat input, Rect rect) {
        m1 = new Mat(input, rect).clone();
        int nw = w1, nh = 1;
        m2 = new Mat(nw, nh, input.type());
        Size s = new Size(nw, nh);
        Imgproc.resize(m1, m2, s);
        double lowestBlue = 255;
        for (int i=0; i<nw; i++) {
            double thisBlue = m2.get(0,i)[0];
            if (thisBlue < lowestBlue) {
                lowestBlue = thisBlue;
            }
        }
        double[] colors = m2.get(0, nw/2);
        double b = colors[0];
        double g = colors[1];
        double r = colors[2];

        double avgColor = Math.sqrt(b * 2 + g * 2 + r * 2);
        ac = avgColor;
        blueDiff = b;

        m1.release();
        m2.release();

        if (lowestBlue == 0) {
            lowestBlue = 0.1; // prevents divide by zero errors
        }
        return lowestBlue;
    }

    public InputExtractor<Double> getAvgColorII() {
        return AvgColorII;
    }

    public InputExtractor<Double> getBlueDiffII() {
        return blueDiffII;
    }

}
