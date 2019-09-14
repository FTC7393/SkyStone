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

public class ObjectDetector {
    private Logger logger;
    private static final String VUFORIA_KEY = "AVfI+4j/////AAABmWhwHR9y1UTwkzlZuCdgpuM+j5Ma5aVefHcpg5Of+kQ5UtZWFVuF3X0n1rsnBLfonFUGc/8M4kr7S6oKGbWftwmeBHAJVCgXOpB/LIfrzPtMpkP7EFzaOLfmm1MB+yBT4R89DqB4Jn6imD919UG6tSZH6aiJ4+EGiqL50qtXTQkZzoFoAJqDdUco/Nr9iCVxLgaBJyBoz4ruB0oDzGXZo4jjf0d9HqJGpmF1oiXDhwB+UcrL7d0vCb3zYosjAEl9Rltf310GnjeTPnsK5aHzv9fa7lctJwnP17F5+SN2PP4QYhLMSKR5IUyBWKlV+UL0uljA/LhLy2656hYQQa2ltv4AcJU9wn1Rt2YHI1B7ynId\n";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    public static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    public static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    HardwareMap hardwareMap;


    public ObjectDetector(HardwareMap hardwareMap){
        this.hardwareMap=hardwareMap;
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        CameraDevice.getInstance().setFlashTorchMode(true) ;
    }


    public static void initThread(final int numCycles, final Telemetry telemetry, final HardwareMap hardwareMap,
                                  final ResultReceiver<Mineral> goldPositionResultReceiver,
                                  final ResultReceiver<Boolean> actResultReceiver,
                                  final ResultReceiver<List<Mineral>> potentialItemRR
                                  ){
        final ObjectDetector objectDetector=new ObjectDetector(hardwareMap);

        //start the init in a new thread
        new Thread(new Runnable() {
            @Override
            public void run() {
                objectDetector.init();
                int maxTries = 5;
                int i = 0;
                int numCyclesDone = 0;
                while (numCyclesDone < numCycles) {
                    if(actResultReceiver.isReady()&&actResultReceiver.getValue()) {
                        if(i>maxTries){
                            goldPositionResultReceiver.setValue(new Mineral());
                            actResultReceiver.clear();
                            i=0;
                            numCyclesDone++;
                        }
                        else{
                            Mineral detection = objectDetector.act(telemetry, potentialItemRR);
                            telemetry.addData("detection", detection);
                            telemetry.addData("i", i);
                            if (detection != null) {
                                i++;
                                if(detection.getType()!= GoldDetector.Detection.NOTHING) {
                                    goldPositionResultReceiver.setValue(detection);
                                    actResultReceiver.clear();
                                    i=0;
                                    numCyclesDone++;
                                }
                            }
                        }

                    }
                    try {
                        Thread.sleep(70);
                    } catch (InterruptedException e) {
                        // Should this cause the thread to exit?
                    }
                }
                objectDetector.end();
            }

        }).start();
    }
    private void init(){
        initVuforia();  // this gets run for evry object that is created.
        initTfod();
        tfod.activate();
    }
    private Mineral act(Telemetry telemetry, ResultReceiver<List<Mineral>> potentialItemRR) {
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            Mineral detection= GoldDetector.findPosition(updatedRecognitions, telemetry, potentialItemRR);
            return detection;
        }
        return null;
    }

    private void end()
    {
        CameraDevice.getInstance().setFlashTorchMode(false) ;

        tfod.shutdown();
    }

}
