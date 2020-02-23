package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Moves off of auto line.
 */
public class AutoBrettV7 extends LowGearAuto {

  public AutoBrettV7() {
    Trajectory forwardPastAutoLine = TrajectoryGenerator.generateTrajectory(
        driveTrain.getPoseListFromPathWeaverJson("Forward"),
        driveTrain.getTrajectoryConfig(false));

    addCommands(
        new InstantCommand(driveTrain::resetOdometry, driveTrain),
        new InstantCommand(driveTrain::zeroEncoder, driveTrain),
        driveTrain.getAutonomousCommandFromTrajectory(forwardPastAutoLine)
    );
  }
}
