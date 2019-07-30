package frc.team1816.controlboard;

import edu.wpi.first.wpilibj.Joystick;
import frc.team1816.Constants;

public class GamepadButtonControlBoard implements IButtonControlBoard {
    private static GamepadButtonControlBoard mInstance = null;

    public static GamepadButtonControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new GamepadButtonControlBoard();
        }

        return mInstance;
    }

    private Joystick mJoystick;

    private GamepadButtonControlBoard() {
        mJoystick = new Joystick(Constants.kButtonGamepadPort);
    }

    @Override
    public boolean getWantsCubeLEDBlink() {
        return mJoystick.getRawButton(8);
    }

}
