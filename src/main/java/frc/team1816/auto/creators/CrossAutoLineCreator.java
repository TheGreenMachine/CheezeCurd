package frc.team1816.auto.creators;

import frc.team1816.AutoFieldState;
import frc.team1816.auto.AutoModeBase;
import frc.team1816.auto.modes.CrossAutoLineMode;

public class CrossAutoLineCreator implements AutoModeCreator {

    // Pre-build trajectories to go left and right
    private CrossAutoLineMode mCrossAutoLineMode = new CrossAutoLineMode();

    @Override
    public AutoModeBase getStateDependentAutoMode(AutoFieldState fieldState) {
        return mCrossAutoLineMode;
    }
}

