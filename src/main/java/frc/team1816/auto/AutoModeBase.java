package frc.team1816.auto;

import edu.wpi.first.wpilibj.DriverStation;
import frc.team1816.auto.actions.Action;

/**
 * An abstract class that is the basis of the robot's autonomous routines. This is implemented in auto modes (which are
 * routines that do actions).
 */
public abstract class AutoModeBase {
    private boolean mActive = false;

    protected abstract void routine() throws AutoModeEndedException;

    void run() {
        mActive = true;

        try {
            routine();
        } catch (AutoModeEndedException e) {
            DriverStation.reportError("AUTO MODE DONE!!!! ENDED EARLY!!!!", false);
            return;
        }

        done();
    }

    private void done() {
        System.out.println("Auto mode done");
    }

    public void stop() {
        mActive = false;
    }

    private boolean isActive() {
        return mActive;
    }

    private boolean isActiveWithThrow() throws AutoModeEndedException {
        if (!isActive()) {
            throw new AutoModeEndedException();
        }
        return true;
    }

    public void runAction(Action action) throws AutoModeEndedException {
        isActiveWithThrow();
        action.start();

        while (isActiveWithThrow() && !action.isFinished()) {
            action.update();
            double mUpdateRate = 1.0 / 50.0;
            long waitTime = (long) (mUpdateRate * 1000.0);

            try {
                Thread.sleep(waitTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        action.done();
    }
}
