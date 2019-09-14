package ftc.evlib.vision.processors;

import android.util.Log;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import ftc.evlib.util.StepTimer;
import ftc.evlib.vision.ImageUtil;

/**
 * This file was made by the electronVolts, FTC team 7393
 * Date Created: 8/29/16.
 *
 * Similar to RGBBeaconProcessor, but uses 0 to 179 as the color instead of RGB
 */
public class HueBeaconProcessor implements ftc.evlib.vision.processors.ImageProcessor<ftc.evlib.vision.processors.BeaconResult> {
    private static final String TAG = "HueBeaconProcessor";
    private static final int THICKNESS = 2;

    //thresholds and parameters to tune the beacon detector
    private static final int GAP = 1;
    private static final int S_MIN = 40;
    private static final int V_MIN = 100; //150;
//  private static final Scalar HSV_MAX = new Scalar(179, 255, 255);

    /**
     * Convert to hsv
     * Threshold black in hsv
     * Find contours (edge detection on binary image)
     * Sort contours to find circles
     * Find 2 circles closest in size (black buttons on the beacon)
     * Rotate the image so the beacon is upright
     *
     * @param startTime the time the frame was received
     * @param rgbaFrame the input image
     * @return a BeaconColorResult object which contains info about the beacon position and colors
     */
    @Override
    public ImageProcessorResult<ftc.evlib.vision.processors.BeaconResult> process(long startTime, Mat rgbaFrame, boolean saveImages) {
        StepTimer stepTimer = new StepTimer(TAG, Log.INFO);
        stepTimer.start();

        if (saveImages) {
            stepTimer.step("save 00");
            //save the raw camera image for logging
            ImageUtil.saveImage(TAG, rgbaFrame, Imgproc.COLOR_RGBA2BGR, "00_camera", startTime);
        }

        stepTimer.step("rgb2hsv");
        //convert image to hsv
        Mat hsv = new Mat();
        Imgproc.cvtColor(rgbaFrame, hsv, Imgproc.COLOR_RGB2HSV);

        stepTimer.step("beaconFinder");
        ftc.evlib.vision.processors.BeaconFinder beaconFinder = new BeaconFinder();
        ImageProcessorResult<BeaconPositionResult> result = beaconFinder.process(startTime, rgbaFrame, saveImages);
        BeaconPositionResult positionResult = result.getResult();
        rgbaFrame = result.getFrame();

        if (positionResult == null) {
            if (saveImages) {
                stepTimer.step("save 02");
                ImageUtil.saveImage(TAG, rgbaFrame, Imgproc.COLOR_RGBA2BGR, "02_circles", startTime);
            }
            return new ImageProcessorResult<>(startTime, rgbaFrame, new ftc.evlib.vision.processors.BeaconResult(new ftc.evlib.vision.processors.BeaconColorResult(), null));
        }

        Rect[] rects = {
                positionResult.getLeftRect(),
                positionResult.getRightRect()
        };

        stepTimer.step("rotate image");
        //rotate the frame and hsv image
        Mat rotationMatrix2D = positionResult.getRotationMatrix2D(); //Imgproc.getRotationMatrix2D(midpoint, foundAngleDegrees, 1);
        Imgproc.warpAffine(rgbaFrame, rgbaFrame, rotationMatrix2D, rgbaFrame.size());
        Imgproc.warpAffine(hsv, hsv, rotationMatrix2D, hsv.size());

        //now that the beacon is found, we need to detect the color of each side

        stepTimer.step("area average");
        Mat subHSV;
        Scalar averageRGB;
        ftc.evlib.vision.processors.BeaconColorResult.BeaconColor[] beaconColors = new ftc.evlib.vision.processors.BeaconColorResult.BeaconColor[2];
        byte[] data = new byte[3];
        long[] hsvAverage = new long[3];

        for (int i = 0; i < 2; i++) {
            //crop left and right regions
            subHSV = new Mat(hsv, rects[i]);

            Log.i(TAG, "region " + i + ": " + subHSV.width() + "x" + subHSV.height());

            for (int x = 0; x < subHSV.width(); x++) {
                for (int y = 0; y < subHSV.height(); y++) {
                    subHSV.get(y, x, data);
                    for (int j = 0; j < 3; j++) {
                        hsvAverage[j] += data[j];
//            if (data[j] < -120)
//            Log.i(TAG, "HSV " + i + "," + j + ": " + data[j]);
                    }
                }
            }

            double area = subHSV.size().area();

            if (area != 0) {
                for (int j = 0; j < 3; j++) {
                    hsvAverage[j] /= area;
                    if (j > 0) {
                        hsvAverage[j] += 128;
                    }
                    Log.i(TAG, "HSV " + i + "," + j + ": " + hsvAverage[j]);
                }
            }

            if (area == 0 || hsvAverage[1] < S_MIN || hsvAverage[2] < V_MIN) {
                beaconColors[i] = ftc.evlib.vision.processors.BeaconColorResult.BeaconColor.UNKNOWN;
                averageRGB = ImageUtil.BLACK;
            } else {
                int hue = (int) ((2 * hsvAverage[0]) % 360);
                if (hue < 0) hue += 360;
                if (hue >= 300 + GAP || hue <= 60 - GAP) {
                    beaconColors[i] = ftc.evlib.vision.processors.BeaconColorResult.BeaconColor.RED;
                } else if (hue >= 60 + GAP && hue <= 180 - GAP) {
                    beaconColors[i] = ftc.evlib.vision.processors.BeaconColorResult.BeaconColor.GREEN;
                } else if (hue >= 180 + GAP && hue <= 300 - GAP) {
                    beaconColors[i] = ftc.evlib.vision.processors.BeaconColorResult.BeaconColor.BLUE;
                } else {
                    beaconColors[i] = ftc.evlib.vision.processors.BeaconColorResult.BeaconColor.UNKNOWN;
                }

                averageRGB = ImageUtil.HSVtoRGB(new Scalar(hue / 2, 255, 255));
            }


            //draw the left and right regions
            Imgproc.rectangle(rgbaFrame, rects[i].br(), rects[i].tl(), averageRGB, THICKNESS);
        }

        if (saveImages) {
            stepTimer.step("save 02");
            ImageUtil.saveImage(TAG, rgbaFrame, Imgproc.COLOR_RGBA2BGR, "02_hue", startTime);
        }
        stepTimer.stop();

        return new ImageProcessorResult<>(startTime, rgbaFrame, new BeaconResult(new BeaconColorResult(beaconColors[0], beaconColors[1]), positionResult));
    }
}
