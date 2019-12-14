package org.firstinspires.ftc.teamcode.SkyStone_Season.Autonomous;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.util.BasicResultReceiver;
import ftc.electronvolts.util.InputExtractor;
import ftc.electronvolts.util.TeamColor;

class ProcessPipeline extends OpenCvPipeline {

    private double blueDiff = 0;
    private double ac = 0;

    private final InputExtractor<StateName> StateNameII = new InputExtractor<StateName>() {
        @Override
        public StateName getValue() {
            return option;
        }
    };
    private final InputExtractor<Double> stoneRatioII = new InputExtractor<Double>() {
        @Override
        public Double getValue() {
            return stoneratio;
        }
    };
    private final BasicResultReceiver<StateName> StateRR;
    private final TeamColor tc;
    private final BasicResultReceiver<Boolean> canUpdateSRR;
    private final int x1 = 75, y1 = 388, w1 = 73, h1 = 43;
    private int x2 = x1 + 200, y2 = 384, w2 = w1, h2 = h1 + 10;
    private Mat m1;
    private Mat m2;
    private final int minStabalizationCycles;
    private int numStabalizationCycles = 0;
    Rect rect1 = new Rect(x1, y1, w1, h1);
    Rect rect2 = new Rect(x2, y2, w2, h2);
    private double blue;
    private double blue2;
    private StateName option;
    private double stoneratio;

    public ProcessPipeline(BasicResultReceiver<StateName> stateRR, int minStabalizationCycles, TeamColor tc, BasicResultReceiver<Boolean> canUpdateSRR) {
        StateRR = stateRR;
        this.minStabalizationCycles = minStabalizationCycles;
        this.tc = tc;
        this.canUpdateSRR = canUpdateSRR;
    }

    @Override
    public Mat processFrame(Mat input) {
        if (!(canUpdateSRR.isReady()) || canUpdateSRR.getValue()) {
            if (numStabalizationCycles > minStabalizationCycles) {
                blue = getLowestAvgBlue(input, rect1);
                blue2 = getLowestAvgBlue(input, rect2);
                if (blue2 == 0) {
                    blue2 = 0.1;
                }
                double ratio = blue / blue2;
                option = getNextStateName(ratio);
                stoneratio = blue / blue2;
                StateRR.setValue(option);
            }
            numStabalizationCycles++;
        }
        Imgproc.rectangle(input, rect1, new Scalar(255, 0, 0), 3);
        Imgproc.rectangle(input, rect2, new Scalar(255, 0, 0), 3);
        return input;
    }

    private StateName getNextStateName(double ratio) {
        StateName opt;
        double maxBlueRatioLeft = 0.5; //all values below this indicate that the dark stone is on the left
        double minBlueRatioRight = 2.0; //all values above this indicate that the dark stone is on the right
        if (tc == TeamColor.BLUE) {
            if (ratio < maxBlueRatioLeft) {
                opt = SkyStoneAutonomous.S.SKYSTONE_MIDDLE;
            } else if (ratio > minBlueRatioRight) {
                opt = SkyStoneAutonomous.S.SKYSTONE_RIGHT;
            } else opt = SkyStoneAutonomous.S.SKYSTONE_LEFT;
        } else {
            if (ratio < maxBlueRatioLeft) {
                opt = SkyStoneAutonomous.S.RED_SKYSTONE_LEFT;
            } else if (ratio > minBlueRatioRight) {
                opt = SkyStoneAutonomous.S.RED_SKYSTONE_MIDDLE;
            } else opt = SkyStoneAutonomous.S.RED_SKYSTONE_RIGHT;
        }
        return opt;
    }


    public InputExtractor<StateName> getStateNameII() {
        return StateNameII;
    }

    public InputExtractor<Double> getStoneRatioII() {
        return stoneRatioII;
    }

    private double avgMiddleBlue(Mat input, Rect rect) {
        m1 = new Mat(input, rect).clone();
        int nw = 25, nh = 1;
        m2 = new Mat(nw, nh, input.type());
        Size s = new Size(nw, nh);
        Imgproc.resize(m1, m2, s);
        double[] colors = m2.get(0, 11);
        double b = colors[0];


        m1.release();
        m2.release();
        return b;
    }

    private double getLowestAvgBlue(Mat input, Rect rect) {
        m1 = new Mat(input, rect).clone();
        int nw = 100, nh = 1;
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
        m1.release();
        m2.release();
        return lowestBlue;
    }

    private double avgBlue(int thisBlueLevel, int runningAverage, int maxVariance, int similarCount) {
        if (Math.abs(thisBlueLevel - runningAverage) < maxVariance) {
            runningAverage = (runningAverage * similarCount + thisBlueLevel) / ++similarCount;
        } else {
            runningAverage = -1;
            similarCount = 0;
        }

        return runningAverage;
    }



}
