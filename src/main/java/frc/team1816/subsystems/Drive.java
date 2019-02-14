package frc.team1816.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.team254.lib.drivers.TalonSRXChecker;
import com.team254.lib.drivers.TalonSRXChecker.CheckerConfig;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.ReflectingCSVWriter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team1816.Constants;
import frc.team1816.RobotState;
import frc.team1816.loops.ILooper;
import frc.team1816.loops.Loop;
import frc.team1816.planners.DriveMotionPlanner;

import java.util.ArrayList;

import static frc.team1816.Robot.factory;

public class Drive extends Subsystem {

    private static final String NAME = "drivetrain";
    private static final double DRIVE_ENCODER_PPR = factory.getConstant(NAME, "encPPR");
    private static Drive mInstance = new Drive();
    // Hardware
    private final IMotorControllerEnhanced mLeftMaster;
    private final IMotorControllerEnhanced mRightMaster;
    private final IMotorController mLeftSlaveA;
    private final IMotorController mRightSlaveA;
    private final IMotorController mLeftSlaveB;
    private final IMotorController mRightSlaveB;
    // Control states
    private DriveControlState mDriveControlState;
    private PigeonIMU mPigeon;
    // Hardware states
    private PeriodicIO mPeriodicIO;
    private boolean mIsBrakeMode;
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;
    private DriveMotionPlanner mMotionPlanner;
    private Rotation2d mGyroOffset = Rotation2d.identity();
    private boolean mOverrideTrajectory = false;
    private static double kD =factory.getConstant(NAME,"kD");

    private final Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            synchronized (Drive.this) {
                setOpenLoop(new DriveSignal(0.05, 0.05));
                setBrakeMode(false);
                startLogging();
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Drive.this) {
                switch (mDriveControlState) {
                    case OPEN_LOOP:
                        break;
                    case PATH_FOLLOWING:
                        updatePathFollower();
                        break;
                    default:
                        System.out.println("Unexpected drive control state: " + mDriveControlState);
                        break;
                }
            }
        }

        @Override
        public void onStop(double timestamp) {
            stop();
            stopLogging();
        }
    };

    private void configureMaster(IMotorControllerEnhanced talon, boolean left) {
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100);
        final ErrorCode sensorPresent = talon.configSelectedFeedbackSensor(FeedbackDevice
                .QuadEncoder, 0, 100); //primary closed-loop, 100 ms timeout
        if (sensorPresent != ErrorCode.OK) {
            DriverStation.reportError("Could not detect " + (left ? "left" : "right") + " encoder: " + sensorPresent, false);
        }
        talon.setInverted(left);
        talon.setSensorPhase(true);
        talon.enableVoltageCompensation(true);
        talon.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        talon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, Constants.kLongCANTimeoutMs);
        talon.configVelocityMeasurementWindow(1, Constants.kLongCANTimeoutMs);
        talon.configClosedloopRamp(Constants.kDriveVoltageRampRate, Constants.kLongCANTimeoutMs);
        talon.configNeutralDeadband(0.04, 0);
    }

    private Drive() {
        mPeriodicIO = new PeriodicIO();

        // Start all Talons in open loop mode.
        mLeftMaster = factory.getMotor(NAME, "leftMain");
        configureMaster(mLeftMaster, true);

        mLeftSlaveA = factory.getMotor(NAME, "leftSlaveOne", "leftMain");
        mLeftSlaveA.setInverted(false);

        mLeftSlaveB = factory.getMotor(NAME, "leftSlaveTwo", "leftMain");
        mLeftSlaveB.setInverted(false);

        mRightMaster = factory.getMotor(NAME, "rightMain");
        configureMaster(mRightMaster, false);

        mRightSlaveA = factory.getMotor(NAME, "rightSlaveOne", "rightMain");
        mRightSlaveA.setInverted(true);

        mRightSlaveB = factory.getMotor(NAME, "rightSlaveTwo", "rightMain");
        mRightSlaveB.setInverted(true);

        reloadGains();

        mPigeon = new PigeonIMU(factory.getConstant(NAME,"pigeonId").intValue());
        mPigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR,10,10);

        setOpenLoop(DriveSignal.NEUTRAL);

        // Force a CAN message across.
        mIsBrakeMode = true;
        setBrakeMode(false);

        mMotionPlanner = new DriveMotionPlanner();

    }

    public static Drive getInstance() {
        return mInstance;
    }

    private static double rotationsToInches(double rotations) {
        return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private static double radiansPerSecondToTicksPer100ms(double rad_s) {
        return rad_s / (Math.PI * 2.0) * DRIVE_ENCODER_PPR / 10.0;
    }

    @Override
    public void registerEnabledLoops(ILooper in) {
        in.register(mLoop);
    }

    /**
     * Configure talons for open loop control
     */
    public synchronized void setOpenLoop(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.OPEN_LOOP) {
            setBrakeMode(false);
            System.out.println("Switching to open loop");
            System.out.println(signal);
            mDriveControlState = DriveControlState.OPEN_LOOP;
            mLeftMaster.configNeutralDeadband(0.04, 0);
            mRightMaster.configNeutralDeadband(0.04, 0);
        }
        mPeriodicIO.left_demand = signal.getLeft();
        mPeriodicIO.right_demand = signal.getRight();
        mPeriodicIO.left_feedforward = 0.0;
        mPeriodicIO.right_feedforward = 0.0;
    }

    /**
     * Configures talons for velocity control
     */
    public synchronized void setVelocity(DriveSignal signal, DriveSignal feedforward) {
        if (mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            // We entered a velocity control state.
            setBrakeMode(true);
            mLeftMaster.selectProfileSlot(0, 0);
            mRightMaster.selectProfileSlot(0, 0);
            mLeftMaster.configNeutralDeadband(0.0, 0);
            mRightMaster.configNeutralDeadband(0.0, 0);

            mDriveControlState = DriveControlState.PATH_FOLLOWING;
        }
        mPeriodicIO.left_demand = signal.getLeft();
        mPeriodicIO.right_demand = signal.getRight();
        mPeriodicIO.left_feedforward = feedforward.getLeft();
        mPeriodicIO.right_feedforward = feedforward.getRight();
    }

    public synchronized void setTrajectory(TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory) {
        if (mMotionPlanner != null) {
            mOverrideTrajectory = false;
            mMotionPlanner.reset();
            mMotionPlanner.setTrajectory(trajectory);
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
        }
    }

    public boolean isDoneWithTrajectory() {
        if (mMotionPlanner == null || mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            return false;
        }
        return mMotionPlanner.isDone() || mOverrideTrajectory;
    }

    private synchronized void setBrakeMode(boolean on) {
        if (mIsBrakeMode != on) {
            mIsBrakeMode = on;
            NeutralMode mode = on ? NeutralMode.Brake : NeutralMode.Coast;
            mRightMaster.setNeutralMode(mode);
            mRightSlaveA.setNeutralMode(mode);
            mRightSlaveB.setNeutralMode(mode);

            mLeftMaster.setNeutralMode(mode);
            mLeftSlaveA.setNeutralMode(mode);
            mLeftSlaveB.setNeutralMode(mode);
        }
    }

    synchronized Rotation2d getHeading() {
        return mPeriodicIO.gyro_heading;
    }

    public double getHeadingDegrees(){
        return getHeading().rotateBy(mGyroOffset).getDegrees();
    }

    public double getDesiredHeading(){
        return mPeriodicIO.path_setpoint.state().getRotation().rotateBy(mGyroOffset).getDegrees();
    }

    public synchronized void setHeading(Rotation2d heading) {
        System.out.println("SET HEADING: " + heading.getDegrees());

        mGyroOffset = heading.rotateBy(Rotation2d.fromDegrees(mPigeon.getFusedHeading()).inverse());
        System.out.println("Gyro offset: " + mGyroOffset.getDegrees());

        mPeriodicIO.gyro_heading = heading;
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Right Drive Distance", mPeriodicIO.right_distance);
        SmartDashboard.putNumber("Right Drive Ticks", mPeriodicIO.right_position_ticks);
        SmartDashboard.putNumber("Left Drive Ticks", mPeriodicIO.left_position_ticks);
        SmartDashboard.putNumber("Left Drive Distance", mPeriodicIO.left_distance);
        SmartDashboard.putNumber("Right Linear Velocity", getRightLinearVelocity());
        SmartDashboard.putNumber("Left Linear Velocity", getLeftLinearVelocity());

        SmartDashboard.putNumber("x err", mPeriodicIO.error.getTranslation().x());
        SmartDashboard.putNumber("y err", mPeriodicIO.error.getTranslation().y());
        SmartDashboard.putNumber("theta err", mPeriodicIO.error.getRotation().getDegrees());
        if (getHeading() != null) {
            SmartDashboard.putNumber("Gyro Heading", getHeading().getDegrees());
        }
        if (mCSVWriter != null) {
            mCSVWriter.write();
        }
    }

    private synchronized void resetEncoders() {
        mLeftMaster.setSelectedSensorPosition(0, 0, 0);
        mRightMaster.setSelectedSensorPosition(0, 0, 0);
        mPeriodicIO = new PeriodicIO();
    }

    @Override
    public void zeroSensors() {
        setHeading(Rotation2d.identity());
        resetEncoders();
    }

    private double getLeftEncoderRotations() {
        return mPeriodicIO.left_position_ticks / DRIVE_ENCODER_PPR;
    }

    private double getRightEncoderRotations() {
        return mPeriodicIO.right_position_ticks / DRIVE_ENCODER_PPR;
    }

    public double getLeftEncoderDistance() {
        return rotationsToInches(getLeftEncoderRotations());
    }

    public double getRightEncoderDistance() {
        return rotationsToInches(getRightEncoderRotations());
    }

    public double getRightVelocityNativeUnits() {
        return mPeriodicIO.right_velocity_ticks_per_100ms;
    }

    double getRightLinearVelocity() {
        return rotationsToInches(getRightVelocityNativeUnits() * 10.0 / DRIVE_ENCODER_PPR);
    }

    public double getLeftVelocityNativeUnits() {
        return mPeriodicIO.left_velocity_ticks_per_100ms;
    }

    public double getLeftVelocityDemand() {
        return mPeriodicIO.left_demand;
    }

    public double getRightVelocityDemand() {
        return mPeriodicIO.right_demand;
    }

    public double getLeftError(){
        return mPeriodicIO.left_error;
    }

    public double getRightError(){
        return mPeriodicIO.right_error;
    }

    double getLeftLinearVelocity() {
        return rotationsToInches(getLeftVelocityNativeUnits() * 10.0 / DRIVE_ENCODER_PPR);
    }

    private void updatePathFollower() {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING) {
            final double now = Timer.getFPGATimestamp();

            DriveMotionPlanner.Output output = mMotionPlanner.update(now, RobotState.getInstance().getFieldToVehicle(now));

            // DriveSignal signal = new DriveSignal(demand.left_feedforward_voltage / 12.0, demand.right_feedforward_voltage / 12.0);

            mPeriodicIO.error = mMotionPlanner.error();
            mPeriodicIO.path_setpoint = mMotionPlanner.setpoint();

            if (!mOverrideTrajectory) {
                setVelocity(new DriveSignal(radiansPerSecondToTicksPer100ms(output.left_velocity), radiansPerSecondToTicksPer100ms(output.right_velocity)),
                        new DriveSignal(output.left_feedforward_voltage / 12.0, output.right_feedforward_voltage / 12.0));

                mPeriodicIO.left_accel = radiansPerSecondToTicksPer100ms(output.left_accel) / 1000.0;
                mPeriodicIO.right_accel = radiansPerSecondToTicksPer100ms(output.right_accel) / 1000.0;
            } else {
                setVelocity(DriveSignal.BRAKE, DriveSignal.BRAKE);
                mPeriodicIO.left_accel = mPeriodicIO.right_accel = 0.0;
            }
        } else {
            DriverStation.reportError("Drive is not in path following state", false);
        }
    }

    private synchronized void reloadGains() {
        reloadTalonGains(mLeftMaster);
        reloadTalonGains(mRightMaster);
    }

    private void reloadTalonGains(IMotorControllerEnhanced talon) {
        talon.config_kP(0, factory.getConstant(NAME,"kP"), Constants.kLongCANTimeoutMs);
        talon.config_kI(0, factory.getConstant(NAME,"kI"), Constants.kLongCANTimeoutMs);
        talon.config_kD(0, kD, Constants.kLongCANTimeoutMs);
        talon.config_kF(0, factory.getConstant(NAME,"kF"), Constants.kLongCANTimeoutMs);
        talon.config_IntegralZone(0, factory.getConstant(NAME,"iZone").intValue(), Constants.kLongCANTimeoutMs);
    }

    @Override
    public void writeToLog() {
    }

    @Override
    public synchronized void readPeriodicInputs() {
        double prevLeftTicks = mPeriodicIO.left_position_ticks;
        double prevRightTicks = mPeriodicIO.right_position_ticks;
        mPeriodicIO.left_position_ticks = mLeftMaster.getSelectedSensorPosition(0);
        mPeriodicIO.right_position_ticks = mRightMaster.getSelectedSensorPosition(0);
        mPeriodicIO.left_velocity_ticks_per_100ms = mLeftMaster.getSelectedSensorVelocity(0);
        mPeriodicIO.right_velocity_ticks_per_100ms = mRightMaster.getSelectedSensorVelocity(0);
        mPeriodicIO.gyro_heading = Rotation2d.fromDegrees(mPigeon.getFusedHeading()).rotateBy(mGyroOffset);
        mPeriodicIO.left_error = mLeftMaster.getClosedLoopError(0);
        mPeriodicIO.right_error = mRightMaster.getClosedLoopError(0);

        double deltaLeftTicks = ((mPeriodicIO.left_position_ticks - prevLeftTicks) / DRIVE_ENCODER_PPR) * Math.PI;
        if (deltaLeftTicks > 0.0) {
            mPeriodicIO.left_distance += deltaLeftTicks * Constants.kDriveWheelDiameterInches;
        } else {
            mPeriodicIO.left_distance += deltaLeftTicks * Constants.kDriveWheelDiameterInches;
        }

        double deltaRightTicks = ((mPeriodicIO.right_position_ticks - prevRightTicks) / DRIVE_ENCODER_PPR) * Math.PI;
        if (deltaRightTicks > 0.0) {
            mPeriodicIO.right_distance += deltaRightTicks * Constants.kDriveWheelDiameterInches;
        } else {
            mPeriodicIO.right_distance += deltaRightTicks * Constants.kDriveWheelDiameterInches;
        }

        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }

        // System.out.println("control state: " + mDriveControlState + ", left: " + mPeriodicIO.left_demand + ", right: " + mPeriodicIO.right_demand);
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mDriveControlState == DriveControlState.OPEN_LOOP) {
            mLeftMaster.set(ControlMode.PercentOutput, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward, 0.0);
            mRightMaster.set(ControlMode.PercentOutput, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward, 0.0);
        } else {
            mLeftMaster.set(ControlMode.Velocity, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward,
                    mPeriodicIO.left_feedforward + kD * mPeriodicIO.left_accel / 1023.0);
            mRightMaster.set(ControlMode.Velocity, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward,
                    mPeriodicIO.right_feedforward + kD * mPeriodicIO.right_accel / 1023.0);
        }
    }

    @Override
    public void checkSystem() {
        boolean leftSide = TalonSRXChecker.CheckTalons(this,
                new ArrayList<>() {
                    {
                        add(new TalonSRXChecker.TalonSRXConfig("left_master", mLeftMaster));
                    }
                }, getTalonCheckerConfig(mLeftMaster));
        boolean rightSide = TalonSRXChecker.CheckTalons(this,
                new ArrayList<>() {
                    {
                        add(new TalonSRXChecker.TalonSRXConfig("right_master", mRightMaster));
                    }
                }, getTalonCheckerConfig(mRightMaster));
    }

    private CheckerConfig getTalonCheckerConfig(IMotorControllerEnhanced talon) {
        return new CheckerConfig() {
            {
                mCurrentFloor = .1;
                mRPMFloor = 160;
                mCurrentEpsilon = .2;
                mRPMEpsilon = 20;
                mRPMSupplier = () -> talon.getSelectedSensorVelocity(0);
            }
        };
    }

    private synchronized void startLogging() {
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/DRIVE-LOGS.csv", PeriodicIO.class);
        }
    }

    private synchronized void stopLogging() {
        if (mCSVWriter != null) {
            mCSVWriter.flush();
            mCSVWriter = null;
        }
    }

    // The robot drivetrain's various states.
    public enum DriveControlState {
        OPEN_LOOP, // open loop voltage control
        PATH_FOLLOWING, // velocity PID control
    }

    public static class PeriodicIO {
        // INPUTS
        int left_position_ticks;
        int right_position_ticks;
        double left_distance;
        double right_distance;
        int left_velocity_ticks_per_100ms;
        int right_velocity_ticks_per_100ms;
        Rotation2d gyro_heading = Rotation2d.identity();
        Pose2d error = Pose2d.identity();
        double left_error;
        double right_error;

        // OUTPUTS
        double left_demand;
        double right_demand;
        double left_accel;
        double right_accel;
        double left_feedforward;
        double right_feedforward;
        TimedState<Pose2dWithCurvature> path_setpoint = new TimedState<>(Pose2dWithCurvature.identity());
    }
}

