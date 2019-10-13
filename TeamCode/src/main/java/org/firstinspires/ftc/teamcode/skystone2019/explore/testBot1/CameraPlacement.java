package org.firstinspires.ftc.teamcode.skystone2019.explore.testBot1;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

public class CameraPlacement {
    private final float cameraForwardDisplacementInches; // how many inches in front of robot center
    private final float cameraVerticalDisplacemetInches; // how many inches above ground
    private final float cameraLeftDisplacementInches; // how many inches left of robot center;
    private final boolean isPhonePortrait;
    private final VuforiaLocalizer.CameraDirection cameraChoice;

    public CameraPlacement(float cameraForwardDisplacementInches, float cameraVerticalDisplacemetInches, float cameraLeftDisplacementInches, boolean isPhonePortrait, VuforiaLocalizer.CameraDirection cameraChoice) {
        this.cameraForwardDisplacementInches = cameraForwardDisplacementInches;
        this.cameraVerticalDisplacemetInches = cameraVerticalDisplacemetInches;
        this.cameraLeftDisplacementInches = cameraLeftDisplacementInches;
        this.isPhonePortrait = isPhonePortrait;
        this.cameraChoice = cameraChoice;
    }

    /** This is for the tank drive test bot */
    public static CameraPlacement createTestBotCP() {
        return new CameraPlacement(2.0f, 4.0f, 0.0f, false, VuforiaLocalizer.CameraDirection.BACK);
    }

    public boolean isPhonePortrait() {
        return isPhonePortrait;
    }

    public VuforiaLocalizer.CameraDirection getCameraChoice() {
        return cameraChoice;
    }

    public float getCameraForwardDisplacementInches() {
        return cameraForwardDisplacementInches;
    }

    public float getCameraVerticalDisplacemetInches() {
        return cameraVerticalDisplacemetInches;
    }

    public float getCameraLeftDisplacementInches() {
        return cameraLeftDisplacementInches;
    }

}
