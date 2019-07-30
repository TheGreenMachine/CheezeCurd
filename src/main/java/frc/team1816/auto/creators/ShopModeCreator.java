package frc.team1816.auto.creators;

import frc.team1816.AutoFieldState;
import frc.team1816.auto.AutoModeBase;
import frc.team1816.auto.modes.ShopMode;

public class ShopModeCreator implements AutoModeCreator {
    private ShopMode mShopMode = new ShopMode();

    @Override
    public AutoModeBase getStateDependentAutoMode(AutoFieldState fieldState) {
        return mShopMode;
    }
}
