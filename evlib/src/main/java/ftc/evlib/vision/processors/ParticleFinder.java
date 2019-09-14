package ftc.evlib.vision.processors;

import android.util.Log;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import ftc.evlib.util.StepTimer;
import ftc.evlib.vision.ImageUtil;
import ftc.electronvolts.util.Function;

/**
 * This file was made by the electronVolts, FTC team 7393
 * Date Created: 9/26/16
 *
 * An ImageProcessor that finds particles (field object)
 */
public class ParticleFinder implements ftc.evlib.vision.processors.ImageProcessor<List<ftc.evlib.vision.processors.Particle>> {
    private static final String TAG = "ParticleFinder";
    private static final Scalar[] BRIGHT_RGB = {ImageUtil.RED, ImageUtil.GREEN, ImageUtil.BLUE};
    private static final Scalar[] DIM_RGB = {ImageUtil.HSVtoRGB(0, 255, 64), ImageUtil.HSVtoRGB(120, 255, 64), ImageUtil.HSVtoRGB(240, 255, 64)};
//    private static String[] colorNames = {"red", "green", "blue"};

    private static final int BLUR_AMOUNT = 7; //used to blur the binary image
    private static final int BLUR_MAX_V = 80; //post-blur hsv value threshold
    //    private static final double FRAME_AREA_SCALE = 1; //E-4; //scaling factor for the frame's area
    private static final double MIN_AREA = .005; //min area for circles to be considered
    private static final double MAX_AREA = .1; //max area for circles
    private static final double MIN_RATIO = .7; //min ratio of ellipse sides to be considered a circle
    private static final double MAX_AREA_ERROR = 5; //actual area to the expected area based on y position
    private static final double FIELD_TO_AREA_CONSTANT = 21.735; //10.4176944796;

    private static final Function FIELD_Y_TO_AREA = new Function() {
        @Override
        public double f(double y) {
            return FIELD_TO_AREA_CONSTANT / y / y;
        }
    };
    private static final Function AREA_TO_FIELD_Y = new Function() {
        @Override
        public double f(double area) {
            return Math.sqrt(FIELD_TO_AREA_CONSTANT / area);
        }
    };
    private static final ftc.evlib.vision.processors.Particle.Color[] PARTICLE_COLORS = {ftc.evlib.vision.processors.Particle.Color.RED, ftc.evlib.vision.processors.Particle.Color.BLUE};

    private final StepTimer stepTimer = new StepTimer(TAG, Log.INFO);

    @Override
    public ImageProcessorResult<List<ftc.evlib.vision.processors.Particle>> process(long startTime, Mat rgbaFrame, boolean saveImages) {
        int thickness = rgbaFrame.height() / 256;
        if (thickness < 1) thickness = 1;

        if (saveImages) {
            stepTimer.step("save 00");
            //save the raw camera image for logging
            ImageUtil.saveImage(TAG, rgbaFrame, Imgproc.COLOR_RGBA2BGR, "00_camera", startTime);
        }

        stepTimer.step("rgb2hsv");
        //convert image to hsv
        Mat hsv = new Mat();
        Imgproc.cvtColor(rgbaFrame, hsv, Imgproc.COLOR_RGB2HSV);

        stepTimer.step("average hsv");
        double[] averages = new double[3];
        double[] pixel;

        Size smallHsvSize = new Size(9, 16);
        Mat smallHsv = new Mat();
        Imgproc.resize(hsv, smallHsv, smallHsvSize);

        for (int i = 0; i < smallHsv.width(); i++) {
            for (int j = 0; j < smallHsv.height(); j++) {
                pixel = smallHsv.get(j, i);
                for (int k = 0; k < 3; k++) {
                    averages[k] += pixel[k];
                }
            }
        }

        for (int k = 0; k < 3; k++) {
            averages[k] /= smallHsvSize.area();
        }

        if (saveImages) {
            stepTimer.step("save average");
            ImageUtil.saveImage(TAG, smallHsv, Imgproc.COLOR_HSV2BGR, "00_average", startTime);
        }

//        stepTimer.start();
//        Mat onePixelHsv = new Mat();
//        Imgproc.resize(hsv, onePixelHsv, new Size(1, 1));
//        double[] averages2 = onePixelHsv.get(0, 0);
//        stepTimer.log("1 pixel");

        Log.i(TAG, "image averages HSV: " + Arrays.toString(averages));
//        Log.i(TAG, "1 x 1 averages HSV: " + Arrays.toString(averages2));

//        double averageH = averages[0];
//        double averageS = averages[1];
        double averageV = averages[2];


        //averageV  min V
        //178       10
        //12        1


        double minS = 150; //50
        double minV = (10.0 - 1.0) / (178.0 - 12.0) * (averageV - 12.0) + 1.0;
        if (minV < 0) minV = 0;

        Log.i(TAG, "minV: " + minV);


        //calculate the hsv thresholds
        //the h value goes from 0 to 179
        //the s value goes from 0 to 255
        //the v value goes from 0 to 255

        //the values are stored as a list of min HSV and a list of max HSV
        List<Scalar> thresholdMin = new ArrayList<>();
        List<Scalar> thresholdMax = new ArrayList<>();

//        thresholdMin.add(new Scalar((300) / 2, minS, minV));
//        thresholdMax.add(new Scalar((60) / 2, 255, 255));
//
//        thresholdMin.add(new Scalar((60) / 2, minS, minV));
//        thresholdMax.add(new Scalar((180) / 2, 255, 255));
//
//        thresholdMin.add(new Scalar((180) / 2, minS, minV));
//        thresholdMax.add(new Scalar((300) / 2, 255, 255));

        //larger red range
        thresholdMin.add(new Scalar((304) / 2, minS, minV));
        thresholdMax.add(new Scalar((16) / 2, 255, 255));

        //1-value green range
//        thresholdMin.add(new Scalar((60) / 2, 255, 255));
//        thresholdMax.add(new Scalar((60) / 2, 255, 255));

        // large blue range
        thresholdMin.add(new Scalar((150) / 2, minS, minV));
        thresholdMax.add(new Scalar((300) / 2, 255, 255));

        //make a list of channels that are blank (used for combining binary images)
        List<Mat> rgbaChannels = new ArrayList<>();
        rgbaChannels.add(null);
        rgbaChannels.add(Mat.zeros(hsv.size(), CvType.CV_8UC1));
        rgbaChannels.add(null);
        rgbaChannels.add(Mat.zeros(hsv.size(), CvType.CV_8UC1));

        List<ftc.evlib.vision.processors.Particle> particles = new ArrayList<>();

        Mat maskedImage;
        //loop through red, blue
        for (int c = 0; c < 2; c++) {
            int channel = 0;
            if (c == 1) { //skip green
                channel = 2;
            }
            stepTimer.step("threshold & blur for channel " + channel);
            //apply HSV thresholds to get binary image
            maskedImage = new Mat();
            ImageUtil.hsvInRange(hsv, thresholdMin.get(c), thresholdMax.get(c), maskedImage);

            //blur the image and re-threshold to "de-bounce" the noisy sections
            Imgproc.blur(maskedImage, maskedImage, new Size(BLUR_AMOUNT, BLUR_AMOUNT));
            Imgproc.threshold(maskedImage, maskedImage, BLUR_MAX_V, 255, Imgproc.THRESH_BINARY_INV);

            rgbaChannels.set(channel, maskedImage);

            stepTimer.step("contours for channel " + channel);
            //find contours (edges between red/blue and not red/blue)
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(maskedImage, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(rgbaFrame, contours, -1, ImageUtil.YELLOW, thickness);

//            if (saveImages) {
//                stepTimer.start();
//                //save the threshold image for logging
//                ImageUtil.saveImage(TAG, maskedImage, Imgproc.COLOR_GRAY2BGR, "01_threshold" + channel, startTime);
//                stepTimer.log("save 01:" + channel + " " + PARTICLE_COLORS[c]);
//            }

            stepTimer.step("finding circles");
            double frameArea = rgbaFrame.size().area();
            //loop through the contours to find the circular ones
            for (int i = 0; i < contours.size(); i++) {
                //convert MatOfPoint to MatOfPoint2f
                MatOfPoint2f contour = new MatOfPoint2f();
                contour.fromList(contours.get(i).toList());

                RotatedRect ellipse = null;
                boolean isValid = false;
                //fitting an ellipse requires at least 5 points
                if (contour.height() >= 5) {
                    //fit an ellipse to the contour points
                    ellipse = Imgproc.fitEllipse(contour);

                    //find the ratio of the shortest side to the longest side
                    //by finding the ratio of the width to the height
                    double ratio = ellipse.size.width / ellipse.size.height;
                    //and inverting it if it is greater than 1
                    if (ratio > 1) ratio = 1 / ratio;

                    //reject ellipses that are not circular enough
                    if (ratio >= MIN_RATIO) {
                        double area = ellipse.size.area();
                        double scaledArea = area / frameArea;

                        //filter out ellipses that are too big or too small
                        if (scaledArea >= MIN_AREA && scaledArea <= MAX_AREA) {
                            double radius = (ellipse.size.height + ellipse.size.width) / 4;
                            Point bottomCamera = new Point(ellipse.center.x, ellipse.center.y + radius);
                            Imgproc.circle(rgbaFrame, bottomCamera, thickness * 2, ImageUtil.BLACK, thickness);
                            Point bottomCameraScaled = new Point(bottomCamera.x / rgbaFrame.width(), bottomCamera.y / rgbaFrame.height());
                            Point bottomField = FieldTransform.imagePointToFieldPoint(bottomCameraScaled);

                            double expectedArea = FIELD_Y_TO_AREA.f(bottomField.y);
                            double areaError = Math.abs(scaledArea - expectedArea) / expectedArea;

                            double fieldYFromArea = AREA_TO_FIELD_Y.f(scaledArea);

                            Log.i(TAG, "[===========================================]");
                            Log.i(TAG, "Ellipse Area: " + area);
                            Log.i(TAG, "Pi R^2 Area: " + Math.PI * radius * radius);
                            Log.i(TAG, "Scaled Area: " + scaledArea);
                            Log.i(TAG, "bottomCamera: " + bottomCamera);
                            Log.i(TAG, "bottomCameraScaled: " + bottomCameraScaled);
                            Log.i(TAG, "bottomField: " + bottomField);
                            Log.i(TAG, "fieldYFromArea: " + fieldYFromArea);
                            Log.i(TAG, "Expected Area: " + expectedArea);
                            Log.i(TAG, "Area Error: " + areaError);

                            if (areaError <= MAX_AREA_ERROR) {
                                isValid = true;
                                //add the particle to the list of results
                                particles.add(new Particle(bottomField, PARTICLE_COLORS[c]));
                            }
                        }
                    }
                }

                //draw the ellipse if it was found
                if (ellipse != null) {
                    if (isValid) {
                        //if it is valid draw it in bright red/blue
                        Imgproc.ellipse(rgbaFrame, ellipse, BRIGHT_RGB[channel], thickness);
                    } else {
                        //if it is not valid, draw it in dark red/dark blue
                        Imgproc.ellipse(rgbaFrame, ellipse, DIM_RGB[channel], thickness);

                    }
                }
            }
//            if (saveImages) {
//                stepTimer.start();
//                //save the raw camera image for logging
//                ImageUtil.saveImage(TAG, maskedImage, Imgproc.COLOR_RGBA2BGR, "02_"+colorNames[channel], startTime);
//                stepTimer.log("save 02");
//            }

        }
        if (saveImages) {
            stepTimer.step("save 03");
            //save the threshold image for logging
            ImageUtil.saveImage(TAG, rgbaFrame, Imgproc.COLOR_RGBA2BGR, "03_final", startTime);

            stepTimer.step("save 04");
            //merge the 3 binary images into one
            Core.merge(rgbaChannels, rgbaFrame);
            ImageUtil.saveImage(TAG, rgbaFrame, Imgproc.COLOR_RGBA2BGR, "04_threshold", startTime);
        }

        stepTimer.stop();
//        Collections.sort(particles);

        Log.i(TAG, "num particles: " + particles.size());
        Log.i(TAG, "particles: " + particles);

        return new ImageProcessorResult<>(startTime, rgbaFrame, particles);
//        if (particles.size() > 0) {
//            Collections.sort(particles);
//            Log.i(TAG, "particle[0]: " + particles.get(0));
//            return new ImageProcessorResult<>(startTime, rgbaFrame, particles.get(0));
//        } else {
//            return new ImageProcessorResult<>(startTime, rgbaFrame, null);
//        }
    }
}
