package frc.team1816.auto.creators;

import frc.team1816.AutoFieldState;
import frc.team1816.auto.AutoModeBase;
import frc.team1816.auto.modes.PIDMode;

public class PIDModeCreator implements AutoModeCreator {
    private PIDMode mMode = new PIDMode();

    @Override
    public AutoModeBase getStateDependentAutoMode(AutoFieldState fieldState) {
        return mMode;
    }
}
