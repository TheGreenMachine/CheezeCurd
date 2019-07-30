package frc.team1816.auto.modes;

import com.team254.lib.geometry.Translation2d;
import frc.team1816.auto.AutoModeBase;
import frc.team1816.auto.AutoModeEndedException;
import frc.team1816.auto.actions.*;
import frc.team1816.paths.TrajectoryGenerator;
import frc.team1816.subsystems.LED;

import java.util.Arrays;

public class LivingRoomMode extends AutoModeBase {

    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    private DriveTrajectory mDriveToStairs;

    public LivingRoomMode() {
        var trajectory = mTrajectoryGenerator.getTrajectorySet().centerStartToStairs;
        mDriveToStairs = new DriveTrajectory(trajectory, true);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new WaitAction(.5));
        runAction(new LEDAction(LED.WantedAction.DISPLAY_INTAKE));
        runAction(new ParallelAction(
            Arrays.asList(
                mDriveToStairs,
                new SeriesAction(
                    Arrays.asList(
                        new WaitUntilInsideRegion(new Translation2d(75, 0), new Translation2d(150, 100), false),
                        new LEDAction(LED.WantedAction.DISPLAY_WANTS_CUBE)
                    )
                ),
                new SeriesAction(
                    Arrays.asList(
                        new WaitUntilInsideRegion(new Translation2d(125, 0), new Translation2d(150, 100), false),
                        new LEDAction(LED.WantedAction.DISPLAY_INTAKE)
                    )
                )
            )
        ));
    }
}
