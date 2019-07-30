package frc.team1816.controlboard;

import edu.wpi.first.wpilibj.Joystick;

public class MainButtonBoard implements IButtonControlBoard {
    private static MainButtonBoard mInstance = null;

    public static MainButtonBoard getInstance() {
        if (mInstance == null) {
            mInstance = new MainButtonBoard();
        }
        return mInstance;
    }

    private final Joystick mButtonBoard;

    private MainButtonBoard() {
        mButtonBoard = new Joystick(0);
    }

    @Override
    public boolean getWantsCubeLEDBlink() {
        return mButtonBoard.getRawButton(2);
    }

}

