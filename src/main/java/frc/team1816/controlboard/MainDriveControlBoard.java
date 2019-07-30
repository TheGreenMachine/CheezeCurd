package frc.team1816.controlboard;

import edu.wpi.first.wpilibj.Joystick;

public class MainDriveControlBoard implements IDriveControlBoard {
    private static MainDriveControlBoard mInstance = null;

    public static MainDriveControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new MainDriveControlBoard();
        }
        return mInstance;
    }

    private final Joystick mThrottleStick;
    private final Joystick mTurnStick;

    private MainDriveControlBoard() {
        mThrottleStick = new Joystick(0);
        mTurnStick = new Joystick(0);
    }

    @Override
    public double getThrottle() {
        return mThrottleStick.getRawAxis(1);
    }

    @Override
    public double getTurn() {
        return -mTurnStick.getRawAxis(0);
    }

    @Override
    public boolean getQuickTurn() {
        return mTurnStick.getRawButton(1);
    }

}
