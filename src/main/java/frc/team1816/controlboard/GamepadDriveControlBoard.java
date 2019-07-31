package frc.team1816.controlboard;

import edu.wpi.first.wpilibj.Joystick;
import frc.team1816.Constants;

public class GamepadDriveControlBoard implements IDriveControlBoard {
    private static GamepadDriveControlBoard mInstance = null;

    public static GamepadDriveControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new GamepadDriveControlBoard();
        }

        return mInstance;
    }

    private Joystick mJoystick;

    private GamepadDriveControlBoard() {
        mJoystick = new Joystick(Constants.kDriveGamepadPort);
    }

    @Override
    public double getThrottle() {
        return -mJoystick.getRawAxis(1);
    }

    @Override
    public double getTurn() {
        return mJoystick.getRawAxis(4);
    }

    @Override
    public boolean getQuickTurn() {
        return mJoystick.getRawButton(6);
    }

}
