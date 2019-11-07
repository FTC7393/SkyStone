package org.firstinspires.ftc.teamcode.SkyStone_Season;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import ftc.electronvolts.util.BasicResultReceiver;
import ftc.electronvolts.util.Function;
import ftc.electronvolts.util.InputExtractor;
import ftc.electronvolts.util.files.Logger;
import ftc.evlib.opmodes.AbstractTeleOp;

@TeleOp(name = "EasyOpenCVTele")
public class EasyOpenCVTestBot extends AbstractTeleOp<TestBotRobotCfg> {
    OpenCvCamera phoneCam;
    private BasicResultReceiver<Boolean> rr = new BasicResultReceiver<>();
    int x,y,w,h;
    static int xStatic, yStatic, wStatic, hStatic;
    public boolean isLocked = false;
    InputExtractor<Integer> xii = new InputExtractor<Integer>() {
        @Override
        public Integer getValue() {
            return x;
        }
    };

    InputExtractor<Integer> yii = new InputExtractor<Integer>() {
        @Override
        public Integer getValue() {
            return y;
        }
    };

    InputExtractor<Integer> wii = new InputExtractor<Integer>() {
        @Override
        public Integer getValue() {
            return w;
        }
    };

    InputExtractor<Integer> hii = new InputExtractor<Integer>() {
        @Override
        public Integer getValue() {
            return h;
        }
    };


//        public void runOpMode()
//        {
    /*
     * Instantiate an OpenCvCamera object for the camera we'll be using.
     * In this sample, we're using the phone's internal camera. We pass it a
     * CameraDirection enum indicating whether to use the front or back facing
     * camera, as well as the view that we wish to use for camera monitor (on
     * the RC phone). If no camera monitor is desired, use the alternate
     * single-parameter constructor instead (commented out below)
     */

    // OR...  Do Not Activate the Camera Monitor View
    //phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

    /*
     * Open the connection to the camera device
     */

    /*
     * Specify the image processing pipeline we wish to invoke upon receipt
     * of a frame from the camera. Note that switching pipelines on-the-fly
     * (while a streaming session is in flight) *IS* supported.
     */

    /*
     * Tell the camera to start streaming images to us! Note that you must make sure
     * the resolution you specify is supported by the camera. If it is not, an exception
     * will be thrown.
     *
     * Also, we specify the rotation that the camera is used in. This is so that the image
     * from the camera sensor can be rotated such that it is always displayed with the image upright.
     * For a front facing camera, rotation is defined assuming the user is looking at the screen.
     * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
     * away from the user.
     */

    /*
     * Wait for the user to press start on the Driver Station
     */
//            waitForStart();

    //            while (opModeIsActive())

        /*
         * Send some stats to the telemetry
         */

        /*
         * The viewport (if one was specified in the constructor) can also be dynamically "paused"
         * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
         * when you need your vision pipeline running, but do not require a live preview on the
         * robot controller screen. For instance, this could be useful if you wish to see the live
         * camera preview as you are initializing your robot, but you no longer require the live
         * preview after you have finished your initialization process; pausing the viewport does
         * not stop running your pipeline.
         *
         * The "if" statements below will pause the viewport if the "X" button on gamepad1 is pressed,
         * and resume the viewport if the "Y" button on gamepad1 is pressed.
         */

        /*
         * For the purposes of this sample, throttle ourselves to 10Hz loop to avoid burning
         * excess CPU cycles for no reason. (By default, telemetry is only sent to the DS at 4Hz
         * anyway). Of course in a real OpMode you will likely not want to do this.
         */
//                sleep(100);



    @Override
    protected Function getJoystickScalingFunction() {
        return null;
    }

    @Override
    protected TestBotRobotCfg createRobotCfg() {
        return new TestBotRobotCfg(hardwareMap);
    }

    @Override
    protected Logger createLogger() {
        return null;
    }

    @Override
    protected void setup() {
        Runnable r = new Runnable() {
            @Override
            public void run() {
                int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
                phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
                phoneCam.openCameraDevice();
                phoneCam.setPipeline(new SamplePipeline(xii, yii, wii, hii));
                phoneCam.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
                rr.setValue(true);
            }
        };
        Thread t = new Thread(r);
        t.start();

    }

    @Override
    protected void setup_act() {

    }

    @Override
    protected void go() {

    }


    @Override
    protected void act() {
        if(!rr.isReady()) {
            return;
        }
        telemetry.addData("Frame Count", phoneCam.getFrameCount());
        telemetry.addData("FPS", String.format("%.2f", phoneCam.getFps()));
        telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
        telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
        telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
        telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());
        telemetry.addData("isLocked", isLocked);
        telemetry.update();

        /*
         * NOTE: stopping the stream from the camera early (before the end of the OpMode
         * when it will be automatically stopped for you) *IS* supported. The "if" statement
         * below will stop streaming from the camera when the "A" button on gamepad 1 is pressed.
         */
        if (gamepad1.a) {
            /*
             * IMPORTANT NOTE: calling stopStreaming() will indeed stop the stream of images
             * from the camera (and, by extension, stop calling your vision pipeline). HOWEVER,
             * if the reason you wish to stop the stream early is to switch use of the camera
             * over to, say, Vuforia or TFOD, you will also need to call closeCameraDevice()
             * (commented out below), because according to the Android Camera API documentation:
             *         "Your application should only have one Camera object active at a time for
             *          a particular hardware camera."
             *
             * NB: calling closeCameraDevice() will internally call stopStreaming() if applicable,
             * but it doesn't hurt to call it anyway, if for no other reason than clarity.
             *
             * NB2: if you are stopping the camera stream to simply save some processing power
             * (or battery power) for a short while when you do not need your vision pipeline,
             * it is recommended to NOT call closeCameraDevice() as you will then need to re-open
             * it the next time you wish to activate your vision pipeline, which can take a bit of
             * time. Of course, this comment is irrelevant in light of the use case described in
             * the above "important note".
             */
            phoneCam.stopStreaming();
            //webcam.closeCameraDevice();

        }
        if (gamepad1.dpad_left){
            x-=1;
            if(x<=0){
                x=0;
            }
        }
        if (gamepad1.dpad_right) {
            x+=1;
        }
        if (gamepad1.dpad_up) {
            y+=1;
        }
        if(gamepad1.dpad_down) {
            y-=1;
            if(y<=0) {
                y=0;
            }
        }
        double threshold = 0.1;
        if(gamepad1.right_stick_x > threshold) {
            w+=1;
        }
        if(gamepad1.right_stick_x < -threshold) {
            w-=1;
            if(w<=0) {
                w = 0;
            }
        }
        if(gamepad1.right_stick_y > threshold) {
            h+=1;
        }
        if(gamepad1.right_stick_y < -threshold) {
            h-=1;
            if(h==0) {
                h = 0;
            }
        }
        if(gamepad1.left_stick_button) {
            xStatic = x;
            yStatic = y;
            wStatic = w;
            hStatic = h;
            isLocked = true;
        }


    }

    @Override
    protected void end() {

    }
}

/*
 * An example image processing pipeline to be run upon receipt of each frame from the camera.
 * Note that the processFrame() method is called serially from the frame worker thread -
 * that is, a new camera frame will not come in while you're still processing a previous one.
 * In other words, the processFrame() method will never be called multiple times simultaneously.
 *
 * However, the rendering of your processed image to the viewport is done in parallel to the
 * frame worker thread. That is, the amount of time it takes to render the image to the
 * viewport does NOT impact the amount of frames per second that your pipeline can process.
 *
 * IMPORTANT NOTE: this pipeline is NOT invoked on your OpMode thread. It is invoked on the
 * frame worker thread. This should not be a problem in the vast majority of cases. However,
 * if you're doing something weird where you do need it synchronized with your OpMode thread,
 * then you will need to account for that accordingly.
 */

class SamplePipeline extends OpenCvPipeline {

    EasyOpenCVTestBot easyCV = new EasyOpenCVTestBot();




    /*
     * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
     * highly recommended to declare them here as instance variables and re-use them for
     * each invocation of processFrame(), rather than declaring them as new local variables
     * each time through processFrame(). This removes the danger of causing a memory leak
     * by forgetting to call mat.release(), and it also reduces memory pressure by not
     * constantly allocating and freeing large chunks of memory.
     */
    private final InputExtractor<Integer> xii;
    private final InputExtractor<Integer> yii;
    private final InputExtractor<Integer> wii;
    private final InputExtractor<Integer> hii;



    public SamplePipeline(InputExtractor<Integer> xii, InputExtractor<Integer> yii, InputExtractor<Integer> wii, InputExtractor<Integer> hii) {
        this.xii = xii;
        this.yii = yii;
        this.wii = wii;
        this.hii = hii;
    }





    @Override
    public Mat processFrame(Mat input) {
        /*
         * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
         * will only dereference to the same image for the duration of this particular
         * invocation of this method. That is, if for some reason you'd like to save a copy
         * of this particular frame for later use, you will need to either clone it or copy
         * it to another Mat.
         */

        /*
         * Draw a simple box around the middle 1/2 of the entire frame
         */
        Imgproc.rectangle(
                input,
                new Point(
                        input.cols() / 4,
                        input.rows() / 4),
                new Point(
                        input.cols() * (3f / 4f),
                        input.rows() * (3f / 4f)),
                new Scalar(0, 255, 0), 4);


        Imgproc.rectangle(input, new Point(xii.getValue(), yii.getValue()),
                new Point(xii.getValue()+wii.getValue(), yii.getValue()+hii.getValue()), new Scalar(255, 0, 0), 4);



        if (easyCV.isLocked) {
            Imgproc.rectangle(input, new Point(easyCV.xStatic, easyCV.yStatic), new Point(easyCV.xStatic+easyCV.wStatic,
                            easyCV.yStatic+easyCV.hStatic), new Scalar(0, 0, 255), 4);
        }




        /**
         * NOTE: to see how to get data from your pipeline to your OpMode as well as how
         * to change which stage of the pipeline is rendered to the viewport when it is
         * tapped, please see {@link PipelineStageSwitchingExample}
         */

        return input;
    }
}


