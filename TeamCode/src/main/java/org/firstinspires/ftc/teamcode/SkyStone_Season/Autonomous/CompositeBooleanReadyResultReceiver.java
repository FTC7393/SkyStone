package org.firstinspires.ftc.teamcode.SkyStone_Season.Autonomous;

import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.util.BasicResultReceiver;
import ftc.electronvolts.util.ResultReceiver;

class CompositeBooleanReadyResultReceiver implements ResultReceiver<Boolean> {
    private final ResultReceiver<?> rr1;
    private final ResultReceiver<?> rr2;
    private boolean ready = false;
    public CompositeBooleanReadyResultReceiver(ResultReceiver<?> rr1,
                                               ResultReceiver<?> rr2) {
        this.rr1 = rr1;
        this.rr2 = rr2;
    }

    @Override
    public boolean isReady() {
        if (!ready) {
            // only check for ready if it's not ready now.
            // So once it becomes ready, it will latch in that state (until
            /// the clear() method is called())
            ready = rr1.isReady() && rr1.isReady();
        }
        return ready;
    }

    @Override
    public Boolean getValue() {
        return ready;
    }

    @Override
    public void setValue(Boolean value) {
        ready = value;
    }

    @Override
    public void clear() {
        ready = false;
    }
}
