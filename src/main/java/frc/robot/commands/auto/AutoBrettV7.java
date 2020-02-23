package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Moves past Auto line.
 */
public class AutoBrettV7 extends LowGearAuto {

  public AutoBrettV7() {
    Trajectory a_b_MovePastAutoLine = TrajectoryGenerator.generateTrajectory(
        driveTrain.getPoseListFromPathWeaverJson("Forward"),
        driveTrain.createTrajectoryConfig(true));

    addCommands(
        new InstantCommand(driveTrain::resetOdometry, driveTrain),
        new InstantCommand(driveTrain::zeroEncoder, driveTrain),
        driveTrain.getAutonomousCommandFromTrajectory(a_b_MovePastAutoLine)
    );
  }
}
