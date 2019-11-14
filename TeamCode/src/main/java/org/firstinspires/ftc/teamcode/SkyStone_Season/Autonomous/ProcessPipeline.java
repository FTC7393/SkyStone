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
    int x = 100, y = 100, w = 100, h = 100;
    Mat m1;
    Mat m2;


    @Override
    public Mat processFrame(Mat input) {
        if(threadName.length() == 0) {
            //only set threadName during first time through processFrame
            threadName = Thread.currentThread().getName();
        }

        Rect rect1 = new Rect(x, y, w, h);
        m1 = new Mat(input, rect1).clone();
        int nw = 5, nh = 5;
        m2 = new Mat(nw, nh, input.type());
        Size s = new Size(nw, nh);
        Imgproc.resize(m1, m2, s);
        double[] colors = m2.get(2,2);
        double b = colors[0];
        double g = colors[1];
        double r = colors[2];

        double avgColor = Math.sqrt(b * 2 + g * 2 + r * 2);
        ac = avgColor;
        blueDiff = b;

        Imgproc.rectangle(m1, rect1, new Scalar(255, 0, 0));

        m1.release();
        m2.release();

        return input;
    }

    public InputExtractor<Double> getAvgColorII() {
        return AvgColorII;
    }

    public InputExtractor<Double> getBlueDiffII() {
        return blueDiffII;
    }

}
