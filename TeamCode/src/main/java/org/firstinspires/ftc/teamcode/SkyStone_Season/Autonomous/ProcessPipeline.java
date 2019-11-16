package org.firstinspires.ftc.teamcode.SkyStone_Season.Autonomous;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import ftc.electronvolts.util.InputExtractor;

class ProcessPipeline extends OpenCvPipeline {

    private double blueDiff = 0;
    private double ac = 0;

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
    int x1 = 100, y1 = 100, w1 = 75, h1 = 50;
    int x2 = 225, y2 = 100, w2 = 75, h2 = 50;
    Mat m1;
    Mat m2;
    private final int minStabalizationCycles;
    private int numStabalizationCycles = 0;
    Rect rect1 = new Rect(x1, y1, w1, h1);
    Rect rect2 = new Rect(x2, y2, w2, h2);

    public ProcessPipeline(int minStabalizationCycles) {
        this.minStabalizationCycles = minStabalizationCycles;
    }

    @Override
    public Mat processFrame(Mat input) {

        if (numStabalizationCycles > minStabalizationCycles) {
            avgMiddleBlue(input, rect1);
        }
        Imgproc.rectangle(input, rect1, new Scalar(255, 0, 0), 3);
        Imgproc.rectangle(input, rect2, new Scalar(255, 0, 0), 3);
        numStabalizationCycles++;
        return input;
    }

    public InputExtractor<Double> getAvgColorII() {
        return AvgColorII;
    }

    public InputExtractor<Double> getBlueDiffII() {
        return blueDiffII;
    }

    private double avgMiddleBlue(Mat input, Rect rect) {
        m1 = new Mat(input, rect).clone();
        int nw = 25, nh = 1;
        m2 = new Mat(nw, nh, input.type());
        Size s = new Size(nw, nh);
        Imgproc.resize(m1, m2, s);
        double[] colors = m2.get(2, 2);
        double b = colors[0];
        double g = colors[1];
        double r = colors[2];

        double avgColor = Math.sqrt(b * b + g * g + r * r);
        ac = avgColor;
        blueDiff = b;

        m1.release();
        m2.release();
        return b;
    }

}
