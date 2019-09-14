package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import ftc.electronvolts.util.ResultReceiver;
import ftc.electronvolts.util.files.Logger;


class TfodManagerTest {
    private final HardwareMap hardwareMap;

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private static final String VUFORIA_KEY = "AVfI+4j/////AAABmWhwHR9y1UTwkzlZuCdgpuM+j5Ma5aVefHcpg5Of+kQ5UtZWFVuF3X0n1rsnBLfonFUGc/8M4kr7S6oKGbWftwmeBHAJVCgXOpB/LIfrzPtMpkP7EFzaOLfmm1MB+yBT4R89DqB4Jn6imD919UG6tSZH6aiJ4+EGiqL50qtXTQkZzoFoAJqDdUco/Nr9iCVxLgaBJyBoz4ruB0oDzGXZo4jjf0d9HqJGpmF1oiXDhwB+UcrL7d0vCb3zYosjAEl9Rltf310GnjeTPnsK5aHzv9fa7lctJwnP17F5+SN2PP4QYhLMSKR5IUyBWKlV+UL0uljA/LhLy2656hYQQa2ltv4AcJU9wn1Rt2YHI1B7ynId\n";
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    public static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    public static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    public TfodManagerTest(HardwareMap hwMap) {
        this.hardwareMap = hwMap;
    }

    public void init() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);

        CameraDevice.getInstance().setFlashTorchMode(true);

        tfod.activate();
    }

    public void finalize() {
        CameraDevice.getInstance().setFlashTorchMode(false);
        tfod.shutdown();
    }

    public TFObjectDetector getTfod() {
        return tfod;
    }
}


public class ObjectDetectorTest {
    private Logger logger;

    private final TfodManagerTest tfodMgr;
    private final Telemetry telemetry;
    private final ResultReceiver<List<Mineral>> potentialItemsRR;

    public ObjectDetectorTest(HardwareMap hardwareMap, Telemetry telemetry, ResultReceiver<List<Mineral>> potentialItemsRR) {
        tfodMgr = new TfodManagerTest(hardwareMap);
        this.telemetry = telemetry;
        this.potentialItemsRR = potentialItemsRR;
    }
    public void init() {
        tfodMgr.init();
    }
    public void finalize() {
        tfodMgr.finalize();
    }

        public static void initThread(final int numCycles,
                                      final Telemetry telemetry, final HardwareMap hardwareMap,
                                      final ResultReceiver<Mineral> goldPositionResultReceiver,
                                      final ResultReceiver<Boolean> actResultReceiver,
                                      ResultReceiver<List<Mineral>> potentialItemsRR) {
        final ObjectDetectorTest objectDetector = new ObjectDetectorTest(hardwareMap, telemetry, potentialItemsRR);
        //start the init in a new thread
        new Thread(new Runnable() {
            @Override
            public void run() {
                // only init once!
                objectDetector.init();
                boolean waitingForTrigger = true;

                int maxTries = 40;
                int i = 0;
                int numCyclesDone = 0;
                while (numCyclesDone < numCycles) {
                    if (actResultReceiver.isReady() && actResultReceiver.getValue()) {
                        if (i > maxTries) {
                            goldPositionResultReceiver.setValue(new Mineral());
                            actResultReceiver.clear();
                            i=0;
                            numCyclesDone++;
                        } else {
                            Mineral detection = objectDetector.act();
                            telemetry.addData("detection", detection);
                            telemetry.addData("i", i);
                            if (detection != null) {
                                i++;
                                if (detection.getType() != GoldDetector.Detection.NOTHING) {
                                    goldPositionResultReceiver.setValue(detection);
                                    actResultReceiver.clear();
                                    i=0;
                                    numCyclesDone++;
                                }
                            }
                        }

                    }
                    try {
                        Thread.sleep(100L);
                    } catch (InterruptedException e) {
                        // Should this cause the thread to exit?
                    }
                }
                objectDetector.finalize();
            }

        }).start();
    }

    public Mineral act() {
        List<Recognition> updatedRecognitions = tfodMgr.getTfod().getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            Mineral detection = GoldDetector.findPosition(updatedRecognitions,telemetry, potentialItemsRR);
            return detection;
        }
        return null;
    }
}
