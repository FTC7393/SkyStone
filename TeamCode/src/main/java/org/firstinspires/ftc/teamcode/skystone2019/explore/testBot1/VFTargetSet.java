package org.firstinspires.ftc.teamcode.skystone2019.explore.testBot1;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

import java.util.List;

import ftc.electronvolts.util.Vector2D;


enum TargetName {
    StoneTarget, BlueRearBridge, RedRearBridge, RedFrontBridge, BlueFrontBridge,
    RedPerimeter1, RedPerimeter2, FrontPerimeter1, FrontPerimeter2,
    BluePerimeter1, BluePerimeter2, RearPerimeter1, RearPerimeter2;
}

class VFTarget {
    private final TargetName name;
    private final VuforiaTrackable trackable;
    private final Vector2D navImagePosition;
    private final Vector2D navImageNormal;

    public VFTarget(TargetName name, VuforiaTrackable trackable, Vector2D navImagePosition, Vector2D navImageNormal) {
        this.name = name;
        this.trackable = trackable;
        this.navImagePosition = navImagePosition;
        this.navImageNormal = navImageNormal;
    }

    public TargetName getName() {
        return name;
    }

    public VuforiaTrackable getTrackable() {
        return trackable;
    }

    public Vector2D getNavImagePosition() {
        return navImagePosition;
    }

    public Vector2D getNavImageNormal() {
        return navImageNormal;
    }
}

public class VFTargetSet {



    public static VFTargetSet create(List<VuforiaTrackable> trackables) {
        return null;
    }



}
