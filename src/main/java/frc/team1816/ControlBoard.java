package frc.team1816;

import frc.team1816.controlboard.*;

public class ControlBoard implements IControlBoard {
    private static ControlBoard mInstance = null;

    public static ControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new ControlBoard();
        }
        return mInstance;
    }

    private IDriveControlBoard mDriveControlBoard;
    private IButtonControlBoard mButtonControlBoard;

    private ControlBoard() {
        if (Constants.kUseGamepadForDriving) {
            mDriveControlBoard = GamepadDriveControlBoard.getInstance();
        } else {
            mDriveControlBoard = MainDriveControlBoard.getInstance();
        }

        if (Constants.kUseGamepadForButtons) {
            mButtonControlBoard = GamepadButtonControlBoard.getInstance();
        } else {
            mButtonControlBoard = MainButtonBoard.getInstance();
        }
    }

    @Override
    public double getThrottle() {
        return mDriveControlBoard.getThrottle();
    }

    @Override
    public double getTurn() {
        return mDriveControlBoard.getTurn();
    }

    @Override
    public boolean getQuickTurn() {
        return mDriveControlBoard.getQuickTurn();
    }

    @Override
    public boolean getWantsCubeLEDBlink() {
        return mButtonControlBoard.getWantsCubeLEDBlink();
    }

}

