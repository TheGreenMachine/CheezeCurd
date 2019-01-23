package frc.team1816.auto.modes;

import frc.team1816.auto.AutoModeBase;
import frc.team1816.auto.AutoModeEndedException;
import frc.team1816.auto.actions.DriveTrajectory;
import frc.team1816.auto.actions.WaitAction;
import frc.team1816.paths.TrajectoryGenerator;

public class LivingRoomMode extends AutoModeBase {

    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    private DriveTrajectory mDriveToStairs;

    public  LivingRoomMode(){
        var trajectory = mTrajectoryGenerator.getTrajectorySet().centerStartToStairs;
        System.out.println(trajectory.toString());
        mDriveToStairs = new DriveTrajectory(trajectory,true);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new WaitAction(.5));
        runAction(mDriveToStairs);
    }

}
