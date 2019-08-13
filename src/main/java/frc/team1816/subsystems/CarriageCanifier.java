package frc.team1816.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifierStatusFrame;
import frc.team1816.Constants;
import frc.team1816.Robot;

public class CarriageCanifier extends Subsystem {
    private static final String NAME = "canifier";
    private static CarriageCanifier mInstance;
    private CANifier mCanifier;
    private PeriodicInputs mPeriodicInputs;
    private PeriodicOutputs mPeriodicOutputs;
    private boolean mOutputsChanged;

    private CarriageCanifier() {
        mCanifier = new CANifier(Robot.getFactory().getConstant(NAME,"canId").intValue());
        if(mCanifier.getDeviceID() >= 0) {
            mCanifier.setStatusFramePeriod(CANifierStatusFrame.Status_1_General, 100, Constants.kLongCANTimeoutMs);
            mCanifier.setStatusFramePeriod(CANifierStatusFrame.Status_2_General, 2, Constants.kLongCANTimeoutMs);
        }
        mPeriodicInputs = new PeriodicInputs();
        mPeriodicOutputs = new PeriodicOutputs();

        // Force a first update.
        mOutputsChanged = true;
    }

    public synchronized static CarriageCanifier getInstance() {
        if (mInstance == null) {
            mInstance = new CarriageCanifier();
        }
        return mInstance;
    }

    synchronized void setLEDColor(double red, double green, double blue) {
        if (red != mPeriodicOutputs.r_ || green != mPeriodicOutputs.g_ || blue != mPeriodicOutputs.b_) {
            mPeriodicOutputs.r_ = red;
            mPeriodicOutputs.g_ = green;
            mPeriodicOutputs.b_ = blue;
            mOutputsChanged = true;
        }
    }

    @Override
    public synchronized void readPeriodicInputs() {
        CANifier.PinValues pins = new CANifier.PinValues();
        mCanifier.getGeneralInputs(pins);
        mPeriodicInputs.left_sensor_state_ = pins.SCL;
        mPeriodicInputs.right_sensor_state_ = pins.SDA;
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        // A: Green
        // B: Red
        // C: Blue
        if (mOutputsChanged) {
            mCanifier.setLEDOutput(mPeriodicOutputs.g_, CANifier.LEDChannel.LEDChannelA);
            mCanifier.setLEDOutput(mPeriodicOutputs.r_, CANifier.LEDChannel.LEDChannelB);
            mCanifier.setLEDOutput(mPeriodicOutputs.b_, CANifier.LEDChannel.LEDChannelC);
            mOutputsChanged = false;
        }
    }

    @Override
    public void checkSystem() {
    }

    @Override
    public void outputTelemetry() {

    }

    @Override
    public void stop() {
        mPeriodicOutputs = new PeriodicOutputs();
        mOutputsChanged = true;
        writePeriodicOutputs();
    }

    @Override
    public void zeroSensors() {
        mPeriodicInputs = new PeriodicInputs();
    }

    private static class PeriodicInputs {
        boolean left_sensor_state_;
        boolean right_sensor_state_;
    }

    private static class PeriodicOutputs {
        double r_;
        double g_;
        double b_;
    }
}

