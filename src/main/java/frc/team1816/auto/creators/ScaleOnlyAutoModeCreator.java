package frc.team1816.auto.creators;

import frc.team1816.AutoFieldState;
import frc.team1816.auto.AutoModeBase;
import frc.team1816.auto.modes.FarScaleOnlyMode;
import frc.team1816.auto.modes.NearScaleOnlyMode;

public class ScaleOnlyAutoModeCreator implements AutoModeCreator {

    private final AutoModeBase hardScaleMode;
    private final AutoModeBase easyScaleMode;
    private boolean mRobotStartedOnLeft;

    public ScaleOnlyAutoModeCreator(boolean robotStartedOnLeft) {
        mRobotStartedOnLeft = robotStartedOnLeft;
        hardScaleMode = new FarScaleOnlyMode(robotStartedOnLeft);
        easyScaleMode = new NearScaleOnlyMode(robotStartedOnLeft);
    }

    @Override
    public AutoModeBase getStateDependentAutoMode(AutoFieldState fieldState) {
        System.out.print("Getting ScaleOnlyAutoMode for " + mRobotStartedOnLeft + " / " + fieldState.toString());
        if ((mRobotStartedOnLeft && fieldState.getScaleSide() == AutoFieldState.Side.LEFT) ||
                (!mRobotStartedOnLeft && fieldState.getScaleSide() == AutoFieldState.Side.RIGHT)) {
            return easyScaleMode;
        } else {
            return hardScaleMode;
        }
    }
}

