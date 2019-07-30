package frc.team1816.auto.creators;

import frc.team1816.AutoFieldState;
import frc.team1816.auto.AutoModeBase;

public interface AutoModeCreator {
    AutoModeBase getStateDependentAutoMode(AutoFieldState fieldState);
}

