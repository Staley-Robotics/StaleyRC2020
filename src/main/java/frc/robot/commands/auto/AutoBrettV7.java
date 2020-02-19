package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;

/**
 * Moves past Auto line.
 */
public class AutoBrettV7 extends SequentialCommandGroup {

  private DriveTrain drive;

  public AutoBrettV7() {

    drive = DriveTrain.getInstance();

    Trajectory forwardPastAutoLine = TrajectoryGenerator.generateTrajectory(
        drive.getPoseListFromPathWeaverJson("Forward"),
        drive.getTrajectoryConfig(false));

    addCommands(
        new InstantCommand(drive::resetOdometry, drive),
        new InstantCommand(drive::zeroEncoder, drive),
        drive.getAutonomousCommandFromTrajectory(forwardPastAutoLine)
    );
  }
}
