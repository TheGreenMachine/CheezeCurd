package frc.team1816;

import badlog.lib.BadLog;
import com.edinarobotics.utils.hardware.RobotFactory;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.util.CheesyDriveHelper;
import com.team254.lib.util.CrashTracker;
import com.team254.lib.util.DriveSignal;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team1816.auto.AutoModeBase;
import frc.team1816.auto.AutoModeExecutor;
import frc.team1816.loops.Looper;
import frc.team1816.paths.TrajectoryGenerator;
import frc.team1816.subsystems.*;

import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Date;
import java.util.Optional;

public class Robot extends TimedRobot {
    private Looper mEnabledLooper = new Looper();
    private Looper mDisabledLooper = new Looper();
    private CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();
    private IControlBoard mControlBoard = ControlBoard.getInstance();
    private AutoFieldState mAutoFieldState = AutoFieldState.getInstance();
    private TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    private AutoModeSelector mAutoModeSelector = new AutoModeSelector();
    private BadLog logger;

    private final SubsystemManager mSubsystemManager = new SubsystemManager(
            Arrays.asList(
                    RobotStateEstimator.getInstance(),
                    Drive.getInstance(),
                    CarriageCanifier.getInstance()
            )
    );

    private Drive mDrive = Drive.getInstance();
    private LED mLED = LED.getInstance();

    private AutoModeExecutor mAutoModeExecutor;

    Robot() {
        CrashTracker.logRobotConstruction();
    }

    public static final RobotFactory factory = new RobotFactory(System.getenv("ROBOT_NAME"));

    private double loopStart;

    @Override
    public void robotInit() {
        try {
            @SuppressWarnings("SpellCheckingInspection")
            var logFile = new SimpleDateFormat("DDDHHmm").format(new Date());
            logger = BadLog.init("/home/lvuser/" + logFile + ".bag");
            BadLog.createTopic("Drivetrain/LeftActVel", "NativeUnits", mDrive::getLeftVelocityNativeUnits, "hide", "join:Drivetrain/Velocities");
            BadLog.createTopic("Drivetrain/RightActVel", "NativeUnits", mDrive::getRightVelocityNativeUnits, "hide", "join:Drivetrain/Velocities");
            BadLog.createTopic("Drivetrain/LeftVel", "NativeUnits", mDrive::getLeftVelocityDemand, "hide", "join:Drivetrain/Velocities");
            BadLog.createTopic("Drivetrain/RightVel", "NativeUnits", mDrive::getRightVelocityDemand, "hide", "join:Drivetrain/Velocities");
            BadLog.createTopic("Drivetrain/LeftError", "NativeUnits", mDrive::getLeftError, "hide", "join:Drivetrain/VelocityError");
            BadLog.createTopic("Drivetrain/RightError", "NativeUnits", mDrive::getRightError, "hide", "join:Drivetrain/VelocityError");
            BadLog.createTopic("Drivetrain/LeftDistance", "Inches", mDrive::getLeftEncoderDistance, "hide", "join:Drivetrain/Distance");
            BadLog.createTopic("Drivetrain/RightDistance", "Inches", mDrive::getRightEncoderDistance, "hide", "join:Drivetrain/Distance");
            BadLog.createTopic("Drivetrain/ActualHeading", "Angle", mDrive::getHeadingDegrees, "hide", "join:Drivetrain/Heading");
            BadLog.createTopic("Drivetrain/Heading", "Angle", mDrive::getDesiredHeading, "hide", "join:Drivetrain/Heading");
            BadLog.createTopic("Timings/Looper", "ms", mEnabledLooper::getLastLoop, "hide", "join:Timings");
            BadLog.createTopic("Timings/RobotLoop", "ms", this::getLastLoop, "hide", "join:Timings");
            logger.finishInitialization();

            CrashTracker.logRobotInit();

            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            mSubsystemManager.registerDisabledLoops(mDisabledLooper);
            mLED.registerEnabledLoops(mEnabledLooper);
            mLED.registerEnabledLoops(mDisabledLooper);
            mTrajectoryGenerator.generateTrajectories();
            mAutoModeSelector.updateModeCreator();
            // Set the auto field state at least once.
            mAutoFieldState.setSides(DriverStation.getInstance().getGameSpecificMessage());
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

  private Double getLastLoop() {
    return (Timer.getFPGATimestamp() - loopStart) * 1000;
  }

  @Override
    public void disabledInit() {
        SmartDashboard.putString("Match Cycle", "DISABLED");

        try {
            CrashTracker.logDisabledInit();
            mEnabledLooper.stop();
            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }

            Drive.getInstance().zeroSensors();
            RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());

            // Reset all auto mode state.
            mAutoModeSelector.reset();
            mAutoModeSelector.updateModeCreator();
            mAutoModeExecutor = new AutoModeExecutor();
            mDisabledLooper.start();
            mLED.setEnableFaults(true);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousInit() {
        SmartDashboard.putString("Match Cycle", "AUTONOMOUS");

        try {
            CrashTracker.logAutoInit();
            mDisabledLooper.stop();

            RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());

            Drive.getInstance().zeroSensors();

            mAutoModeExecutor.start();

            mLED.setEnableFaults(false);
            mEnabledLooper.start();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void teleopInit() {
        SmartDashboard.putString("Match Cycle", "TELEOP");

        try {
            CrashTracker.logTeleopInit();
            mDisabledLooper.stop();
            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }
            mAutoFieldState.disableOverride();
            RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());
            mEnabledLooper.start();
            mLED.setEnableFaults(false);

            mDrive.setVelocity(DriveSignal.NEUTRAL, DriveSignal.NEUTRAL);
            mDrive.setOpenLoop(new DriveSignal(0.05, 0.05));

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testInit() {
        SmartDashboard.putString("Match Cycle", "TEST");

        try {
            System.out.println("Starting check systems.");
            mLED.stop();
            mDisabledLooper.stop();
            mEnabledLooper.stop();
            mDrive.checkSystem();
            mLED.checkSystem();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledPeriodic() {
        loopStart = Timer.getFPGATimestamp();
        SmartDashboard.putString("Match Cycle", "DISABLED");
        try {

            // Poll FMS auto mode info and update mode creator cache
            mAutoFieldState.setSides(DriverStation.getInstance().getGameSpecificMessage());
            mAutoModeSelector.updateModeCreator();

            if (mAutoFieldState.isValid()) {
                Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode(mAutoFieldState);
                if (autoMode.isPresent() && autoMode.get() != mAutoModeExecutor.getAutoMode()) {
                    System.out.println("Set auto mode to: " + autoMode.get().getClass().toString());
                    mAutoModeExecutor.setAutoMode(autoMode.get());
                }
                //System.gc();
            }

            outputToSmartDashboard();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousPeriodic() {
        loopStart = Timer.getFPGATimestamp();
        SmartDashboard.putString("Match Cycle", "AUTONOMOUS");
        outputToSmartDashboard();
    }

    @Override
    public void teleopPeriodic() {
        loopStart = Timer.getFPGATimestamp();
        SmartDashboard.putString("Match Cycle", "TELEOP");

        double throttle = mControlBoard.getThrottle();
        double turn = mControlBoard.getTurn();

        try {

            mDrive.setOpenLoop(mCheesyDriveHelper.cheesyDrive(throttle, turn, mControlBoard.getQuickTurn(), false));


            // LEDs
            if (mControlBoard.getWantsCubeLEDBlink()) {
                mLED.setWantedAction(LED.WantedAction.DISPLAY_WANTS_CUBE);
            } else {
                mLED.setWantedAction(LED.WantedAction.DISPLAY_INTAKE);
            }


            outputToSmartDashboard();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testPeriodic() {
        SmartDashboard.putString("Match Cycle", "TEST");
    }

    private void outputToSmartDashboard() {
        RobotState.getInstance().outputToSmartDashboard();
        Drive.getInstance().outputTelemetry();
        mAutoFieldState.outputToSmartDashboard();
        mEnabledLooper.outputToSmartDashboard();
        mAutoModeSelector.outputToSmartDashboard();
        logger.updateTopics();
        logger.log();
    }
}
