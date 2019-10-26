package org.firstinspires.ftc.teamcode.skystone2019.explore.testBot1;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

import ftc.electronvolts.util.ResultReceiver;
import ftc.electronvolts.util.Vector2D;
import ftc.evlib.hardware.control.TranslationControl;

class TargetFindingTranslationControl implements TranslationControl {
    private static final Vector2D LONG_DISTANCE = new Vector2D(100,100);

    private final int targetIndex;
    private final Vector2D desiredPosition;
    private final ResultReceiver<VuforiaManager> vfManagerResultRecvr;
    private final double gain;
    private final Telemetry telem;

    private Vector2D distanceToTarget = LONG_DISTANCE;
    boolean isInitialized = false;
    TargetTracker tt = null;
    Vector2D translation = null;

    public TargetFindingTranslationControl(int targetIndex, Vector2D desiredPosition, ResultReceiver<VuforiaManager> vfManagerResultRecvr, double gain, Telemetry telem) {
        this.targetIndex = targetIndex;
        this.desiredPosition = desiredPosition;
        this.vfManagerResultRecvr = vfManagerResultRecvr;
        this.gain = gain;
        this.telem = telem;
    }

    @Override
    public boolean act() {
        if (!isInitialized) {
            final VuforiaTrackable desiredNavPic = vfManagerResultRecvr.getValue().getTargetsSkyStone().get(targetIndex); // Rear 2
            VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener) desiredNavPic.getListener();

            tt = new TargetTracker(listener, telem);
            isInitialized = true;
        }
        tt.update();
        Vector2D posn = tt.getPosition();
        double xVelocity = 0;
        double yVelocity = 0;
        if (posn != null) {
            double xDiff = desiredPosition.getX() - posn.getX();
            double yDiff = desiredPosition.getY() - posn.getY();
            xVelocity = xDiff * gain;
            yVelocity = yDiff * gain;
            distanceToTarget = new Vector2D(xDiff, yDiff);
            telem.addData("targt: ","%6.2f   %6.2f",desiredPosition.getX(), desiredPosition.getY());
            telem.addData("displ: ","%6.2f   %6.2f",posn.getX(), posn.getY());
        }
        Vector2D t =  new Vector2D(xVelocity, yVelocity);
        telem.addData("trans: ","%6.2f   %6.2f",xVelocity, yVelocity);
        translation = t; // new Vector2D(0,0);
        return true;
    }

    @Override
    public Vector2D getTranslation() {
        return translation;
    }

    TargetTracker getTargetTracker() {
        return tt;
    }

    double getDistanceToTarget() {
        return distanceToTarget.getLength();
    }
}
