package frc.team1816.auto.modes;

import com.team254.lib.geometry.Translation2d;
import frc.team1816.auto.AutoConstants;
import frc.team1816.auto.AutoModeBase;
import frc.team1816.auto.AutoModeEndedException;
import frc.team1816.auto.actions.*;
import frc.team1816.paths.TrajectoryGenerator;
import frc.team1816.states.SuperstructureConstants;

import java.util.Arrays;

public class SimpleSwitchMode extends AutoModeBase {

    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    final boolean mStartedLeft;
    private DriveTrajectory mStartToSwitch;

    private DriveTrajectory mSwitchToPyramidCube;
    private DriveTrajectory mSwitchToPyramidCube1;
    private DriveTrajectory mSwitchToPyramidCube2;

    private DriveTrajectory mPyramidCubeToSwitch;
    private DriveTrajectory mPyramidCube1ToSwitch;
    private DriveTrajectory mPyramidCube2ToCenterField;

    private double mPyramidCubeWaitTime, mPyramidCube1WaitTime, mStartCubeWaitTime, mPyramidCubeClampTime;

    public SimpleSwitchMode(boolean driveToLeftSwitch) {
        mStartedLeft = driveToLeftSwitch;

        if (mStartedLeft) {
            mStartToSwitch = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().simpleStartToLeftSwitch, true);
            mStartCubeWaitTime = mTrajectoryGenerator.getTrajectorySet().simpleStartToLeftSwitch.getLastState().t() - 0.2;
        } else {
            mStartToSwitch = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().simpleStartToRightSwitch, true);
            mStartCubeWaitTime = mTrajectoryGenerator.getTrajectorySet().simpleStartToRightSwitch.getLastState().t() - 0.2;
        }

        mSwitchToPyramidCube = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().switchToPyramidCube.get(mStartedLeft));
        mSwitchToPyramidCube1 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().switchToPyramidCube1.get(mStartedLeft));
        mSwitchToPyramidCube2 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().switchToPyramidCube2.get(mStartedLeft));

        mPyramidCubeToSwitch = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().pyramidCubeToSwitch.get(mStartedLeft));
        mPyramidCube1ToSwitch = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().pyramidCube1ToSwitch.get(mStartedLeft));
        mPyramidCube2ToCenterField = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().pyramidCube2ToCenterField.get(mStartedLeft));

        mPyramidCubeClampTime = mTrajectoryGenerator.getTrajectorySet().switchToPyramidCube1.get(mStartedLeft).getLastState().t() - 0.15;
        mPyramidCubeWaitTime = mTrajectoryGenerator.getTrajectorySet().pyramidCubeToSwitch.get(mStartedLeft).getLastState().t() - 0.2;
        mPyramidCube1WaitTime = mTrajectoryGenerator.getTrajectorySet().pyramidCube1ToSwitch.get(mStartedLeft).getLastState().t() - 0.2;
    }


    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Simple switch");

        //Score first cube
        runAction(new ParallelAction(
                Arrays.asList(
                        mStartToSwitch,
                        new SetSuperstructurePosition(SuperstructureConstants.kSwitchHeightBackwards, SuperstructureConstants.kStowedPositionAngle, true),
                        new SeriesAction(
                                Arrays.asList(
                                        new WaitAction(mStartCubeWaitTime),
                                        new ShootCube(AutoConstants.kMediumShootPower)
                                )
                        )
                )
        ));

        // Get second cube
        runAction(new ParallelAction(
                Arrays.asList(
                        mSwitchToPyramidCube,
                        new SetSuperstructurePosition(SuperstructureConstants.kIntakePositionHeight, SuperstructureConstants.kIntakePositionAngle, true),
                        new SetIntaking(false, false)
                )
        ));
        runAction(new WaitAction(AutoConstants.kWaitForCubeTime));

        //Score second cube
        runAction(new ParallelAction(
                Arrays.asList(
                        mPyramidCubeToSwitch,
                        new SeriesAction(
                                Arrays.asList(
                                        new SetIntaking(false, true),
                                        new SetSuperstructurePosition(SuperstructureConstants.kSwitchHeightBackwards, SuperstructureConstants.kStowedPositionAngle, true)
                                )
                        ),
                        new SeriesAction(
                                Arrays.asList(
                                        new WaitAction(mPyramidCubeWaitTime),
                                        new ShootCube(AutoConstants.kMediumShootPower)
                                )
                        )
                )
        ));

        // Get third cube
        runAction(new ParallelAction(
                Arrays.asList(
                        mSwitchToPyramidCube1,
                        new SeriesAction(
                                Arrays.asList(
                                        new WaitUntilInsideRegion(new Translation2d(-1000.0, -50.0), new Translation2d
                                                (1000.0, 50.0), mStartedLeft),
                                        new SetSuperstructurePosition(SuperstructureConstants.kIntakeSecondLevelHeight, SuperstructureConstants.kIntakePositionAngle, true),
                                        new SetIntaking(false, false),
                                        new OpenCloseJawAction(true)
                                )
                        ),
                        new SeriesAction(
                                Arrays.asList(
                                        new WaitAction(mPyramidCubeClampTime),
                                        new OpenCloseJawAction(false)
                                )
                        )
                )
        ));
        runAction(new WaitAction(AutoConstants.kWaitForCubeTime));

        //Score third cube
        runAction(new ParallelAction(
                Arrays.asList(
                        mPyramidCube1ToSwitch,
                        new SeriesAction(
                                Arrays.asList(
                                        new SetIntaking(false, true),
                                        new SetSuperstructurePosition(SuperstructureConstants.kSwitchHeightBackwards, SuperstructureConstants.kStowedPositionAngle, true)
                                )
                        ),
                        new SeriesAction(
                                Arrays.asList(
                                        new WaitAction(mPyramidCube1WaitTime),
                                        new ShootCube(AutoConstants.kMediumShootPower)
                                )
                        )
                )
        ));
    }
}

