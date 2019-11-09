package org.firstinspires.ftc.teamcode.SkyStone_Season;

import org.opencv.core.Mat;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfInt;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import ftc.electronvolts.util.BasicResultReceiver;
import ftc.evlib.util.FileUtil;

public class FileSaver {
    private final String phoneSaveDir;
    private final String filenameBase = "hist";
    boolean isBusy = false;
    private int fileIndexCounter = 0;

    // suggested dirname is "/sdcard/FIRST/histograms"
    public FileSaver(String phoneSaveDir) {
        this.phoneSaveDir = phoneSaveDir;
    }

    public void saveHistogramIfNotBusy(final Mat image) {
        if (isBusy) {
            return;
        }
        isBusy = true;
        Runnable r = new Runnable() {
            @Override
            public void run() {
                MatOfFloat ranges = new MatOfFloat(); //0.0f);
                Mat histResult = new Mat(25);
                MatOfInt channels = new MatOfInt(0);
                Mat mask = new Mat();
                MatOfInt histSize = new MatOfInt(25);
                List<Mat> images = new ArrayList<>();
                images.add(image);
//                Imgproc.calcHist(images, channels, mask, histResult, histSize, ranges);
//                // we expect histResult to have a size of "w" for number of x pixels, and a height of 1 (only got one channel!)
//                String filename =
//                        String.format("%s/%s_%04d.png", phoneSaveDir, filenameBase, fileIndexCounter++);
//
//                File dir = FileUtil.getDir(phoneSaveDir);
//
//                Imgcodecs.imwrite(filename, histResult);
                isBusy = false;
            }
        };
        new Thread(r).start();
    }
}
