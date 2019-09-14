package org.firstinspires.ftc.teamcode.RoverRuckus.Sandbox;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.RoverRuckus.Mineral;
import org.firstinspires.ftc.teamcode.RoverRuckus.RoverRuckusRobotCfg;

import java.util.List;

import evlib.opmodes.AbstractTeleOp;
import ftc.electronvolts.util.BasicResultReceiver;
import ftc.electronvolts.util.Function;
import ftc.electronvolts.util.Functions;
import ftc.electronvolts.util.InputExtractor;
import ftc.electronvolts.util.ResultReceiver;
import ftc.electronvolts.util.files.Logger;
import ftc.electronvolts.util.units.Time;

/**
 * Created by ftc7393 on 10/13/2018.
 */
@TeleOp(name = "Sandbox Teleop")
public class SandboxTeleOp extends AbstractTeleOp<RoverRuckusRobotCfg> {
    public Time getMatchTime() {
        return Time.fromMinutes(2); //teleop is 2 minutes
    }

    double rotationValue;
    double extensionValue;
    ScalingInputExtractor rightY;
    ScalingInputExtractor leftX;
    ScalingInputExtractor rightX;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AVfI+4j/////AAABmWhwHR9y1UTwkzlZuCdgpuM+j5Ma5aVefHcpg5Of+kQ5UtZWFVuF3X0n1rsnBLfonFUGc/8M4kr7S6oKGbWftwmeBHAJVCgXOpB/LIfrzPtMpkP7EFzaOLfmm1MB+yBT4R89DqB4Jn6imD919UG6tSZH6aiJ4+EGiqL50qtXTQkZzoFoAJqDdUco/Nr9iCVxLgaBJyBoz4ruB0oDzGXZo4jjf0d9HqJGpmF1oiXDhwB+UcrL7d0vCb3zYosjAEl9Rltf310GnjeTPnsK5aHzv9fa7lctJwnP17F5+SN2PP4QYhLMSKR5IUyBWKlV+UL0uljA/LhLy2656hYQQa2ltv4AcJU9wn1Rt2YHI1B7ynId\n";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private final ResultReceiver<Boolean> setupDoneRR = new BasicResultReceiver<>();

    final ResultReceiver<List<Mineral>> potentialMineralResultReceiver = new BasicResultReceiver<>();
    ResultReceiver<Boolean> cameraActivationRR = new BasicResultReceiver<>();
    ResultReceiver<Mineral> detectionRR = new BasicResultReceiver<>();

    class ScalingInputExtractor implements InputExtractor<Double> {
        InputExtractor<Double> ext;
        private double factor;

        ScalingInputExtractor(InputExtractor<Double> ext, double f) {
            this.ext = ext;
            this.factor = f;
        }

        @Override
        public Double getValue() {
            return ext.getValue() * factor;
        }

        public void setFactor(double f) {
            this.factor = f;
        }
    }

    @Override
    protected Function getJoystickScalingFunction() {
        return Functions.eBased(5);
    }

    @Override
    protected RoverRuckusRobotCfg createRobotCfg() {
        return new RoverRuckusRobotCfg(hardwareMap);
    }

    @Override
    protected Logger createLogger() {
        return null;
    }

    @Override
    protected void setup() {
//        new Thread(new Runnable() {
//            @Override
//            public void run() {
//                initVuforia();
//                initTfod();
//                tfod.activate();
//                int numCycles = 1000;
//                ObjectDetector.initThread(numCycles,
//                        null, hardwareMap, detectionRR, cameraActivationRR, potentialMineralResultReceiver);
//                setupDoneRR.setValue(true);
//            }
//        }).start();
    }
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        CameraDevice.getInstance().setFlashTorchMode(true) ;


        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);

    }

    @Override
    protected void setup_act() {
        robotCfg.getHanging().unlatch();
        robotCfg.getPhonePan().goToPreset(RoverRuckusRobotCfg.PhonePanPresets.MIDDLE);
    }

    @Override
    protected void go() {


    }

    enum PhoneLocation {
        LEFT, MID, RIGHT;
    }
    PhoneLocation ploc = PhoneLocation.MID; // starts here
    @Override
    protected void act() {
        if (!setupDoneRR.isReady() || setupDoneRR.getValue() == false) {
            return;
        }
        if (driver1.right_bumper.isPressed()) {
            // move it to the right
            if (ploc == PhoneLocation.LEFT) {
                ploc = PhoneLocation.MID;
                robotCfg.getPhonePan().goToPreset(RoverRuckusRobotCfg.PhonePanPresets.MIDDLE);
            } else if (ploc == PhoneLocation.MID || ploc == PhoneLocation.RIGHT) {
                ploc = PhoneLocation.RIGHT;
                robotCfg.getPhonePan().goToPreset(RoverRuckusRobotCfg.PhonePanPresets.RIGHT);
            }
        }
        if (driver1.left_bumper.isPressed()) {
            if (ploc == PhoneLocation.RIGHT) {
                ploc = PhoneLocation.MID;
                robotCfg.getPhonePan().goToPreset(RoverRuckusRobotCfg.PhonePanPresets.MIDDLE);
            } else if (ploc == PhoneLocation.MID || ploc == PhoneLocation.LEFT) {
                ploc = PhoneLocation.LEFT;
                robotCfg.getPhonePan().goToPreset(RoverRuckusRobotCfg.PhonePanPresets.LEFT);
            }
        }

        if (potentialMineralResultReceiver.isReady()) {
            List<Mineral> mlist = potentialMineralResultReceiver.getValue();
            for (Mineral m : mlist) {
                m.showInTelem(telemetry);
            }
        }

    }

    @Override
    protected void end() {
        tfod.shutdown();
    }

}
