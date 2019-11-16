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
    int x1 = 100, y1 = 100, w1 = 100, h1 = 100;
    int x2 = 250, y2 = 100, w2 = 100, h2 = 100;
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

            m1 = new Mat(input, rect1).clone();
            int nw = 5, nh = 5;
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

}
