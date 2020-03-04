package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;

public class AutoBrettV7 extends LowGearAuto{

  public AutoBrettV7() {
    Trajectory a_b_MovePastAutoLine = TrajectoryGenerator.generateTrajectory(
        driveTrain.getPoseListFromPathWeaverJson("Forward"),
        driveTrain.createTrajectoryConfig(true));

    //addCommands
  }
}
