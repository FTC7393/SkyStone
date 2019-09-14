package ftc.evlib.vision.processors;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;

import java.util.List;

/**
 * This file was made by the electronVolts, FTC team 7393
 * Date Created: 1/7/17
 */

public class FieldTransform {
    private static final double HOMOGRAPHY_THRESHOLD = 3;

    static {

        MatOfPoint2f src = new MatOfPoint2f();
        MatOfPoint2f dst = new MatOfPoint2f();
        src.fromArray(
                new Point(0.0355392157, 1 - 0.0854779412),
                new Point(0.3206699346, 1 - 0.3734681373),
                new Point(0.7691993464, 1 - 0.0119485294),
                new Point(0.8488562092, 1 - 0.3183210784)
        );
        dst.fromArray(
                new Point(-12, 33),
                new Point(-12, 84),
                new Point(6, 24),
                new Point(18, 60)
        );
        TRANSFORM = Calib3d.findHomography(src, dst, Calib3d.RANSAC, HOMOGRAPHY_THRESHOLD);
    }

    private static final Mat TRANSFORM;


    public static MatOfPoint2f imagePointToFieldPoint(MatOfPoint2f src) {
        MatOfPoint2f dst = new MatOfPoint2f();

        Core.perspectiveTransform(src, dst, TRANSFORM);
        return dst;
    }

    public static Point imagePointToFieldPoint(Point point) {
        MatOfPoint2f src = new MatOfPoint2f();
        src.fromArray(point);

        return imagePointToFieldPoint(src).toList().get(0);
    }

    public static List<Point> imagePointToFieldPoint(List<Point> point) {
        MatOfPoint2f src = new MatOfPoint2f();
        src.fromList(point);

        return imagePointToFieldPoint(src).toList();
    }

    public static Point[] imagePointToFieldPoint(Point[] point) {
        MatOfPoint2f src = new MatOfPoint2f();
        src.fromArray(point);

        return imagePointToFieldPoint(src).toArray();
    }

/*
    Mat cropped_image = original_image.clone();
    Imgproc.warpPerspective(untouched, cropped_image, perspectiveTransform, new Size(512,512));*/

    public FieldTransform() {
    }
}
