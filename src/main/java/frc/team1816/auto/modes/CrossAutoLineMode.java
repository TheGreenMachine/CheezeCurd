package frc.team1816.auto.modes;

import frc.team1816.auto.actions.OpenLoopDrive;
import frc.team1816.auto.actions.WaitAction;
import frc.team1816.auto.AutoModeBase;
import frc.team1816.auto.AutoModeEndedException;

public class CrossAutoLineMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Cross auto line");
        runAction(new WaitAction(5.0));
        runAction(new OpenLoopDrive(-0.3, -0.3, 5.0, false));
    }
}
