package frc.team1816.auto.actions;

import frc.team1816.subsystems.LED;

public class LEDAction implements Action {

    private LED mLED = LED.getInstance();
    private LED.WantedAction mAction;

    public LEDAction(LED.WantedAction action){
        mAction = action;
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {

    }

    @Override
    public void start() {
        mLED.setWantedAction(mAction);
    }
}
