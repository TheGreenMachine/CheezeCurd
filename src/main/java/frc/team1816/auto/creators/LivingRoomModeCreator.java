package frc.team1816.auto.creators;

import frc.team1816.AutoFieldState;
import frc.team1816.auto.AutoModeBase;
import frc.team1816.auto.modes.LivingRoomMode;

public class LivingRoomModeCreator implements AutoModeCreator {

    private LivingRoomMode mLivingRoomMode = new LivingRoomMode();

    @Override
    public AutoModeBase getStateDependentAutoMode(AutoFieldState fieldState) {
        return mLivingRoomMode;
    }
}
