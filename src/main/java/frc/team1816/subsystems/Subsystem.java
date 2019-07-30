package frc.team1816.subsystems;

import frc.team1816.loops.ILooper;

/**
 * The Subsystem abstract class, which serves as a basic framework for all robot subsystems. Each subsystem outputs
 * commands to SmartDashboard, has a stop routine (for after each match), and a routine to zero all sensors, which helps
 * with calibration.
 * <p>
 * All Subsystems only have one instance (after all, one robot does not have two drivetrains), and functions get the
 * instance of the drivetrain and act accordingly. Subsystems are also a state machine with a desired state and actual
 * state; the robot code will try to match the two states with actions. Each Subsystem also is responsible for
 * initializing all member components at the start of the match.
 *
 * The wpilib timed robot uses a thread.sleep which is inconsistent in its timing.
 */
public abstract class Subsystem {
    public void writeToLog() {
    }

    // Optional design pattern for caching periodic reads to avoid hammering the HAL/CAN.
    public void readPeriodicInputs() {
    }

    // Optional design pattern for caching periodic writes to avoid hammering the HAL/CAN.
    public void writePeriodicOutputs() {
    }

    public abstract void checkSystem();

    public abstract void outputTelemetry();

    public abstract void stop();

    public void zeroSensors() {
    }

    public void registerEnabledLoops(ILooper enabledLooper) {
    }
}