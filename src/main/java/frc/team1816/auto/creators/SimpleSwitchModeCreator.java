package frc.team1816.auto.creators;

import frc.team1816.AutoFieldState;
import frc.team1816.auto.AutoModeBase;
import frc.team1816.auto.modes.SimpleSwitchMode;

public class SimpleSwitchModeCreator implements AutoModeCreator {

    // Pre build trajectories to go left and right
    private SimpleSwitchMode mGoLeftMode = new SimpleSwitchMode(true);
    private SimpleSwitchMode mGoRightMode = new SimpleSwitchMode(false);

    @Override
    public AutoModeBase getStateDependentAutoMode(AutoFieldState fieldState) {
        if (fieldState.getOurSwitchSide() == AutoFieldState.Side.LEFT) {
            return mGoLeftMode;
        } else {
            return mGoRightMode;
        }
    }
}

