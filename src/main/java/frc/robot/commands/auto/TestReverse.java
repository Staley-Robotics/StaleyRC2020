package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveTrain;

public class TestReverse extends SequentialCommandGroup {

  private DriveTrain drive;

  private Trajectory trajectoryForward;
  private Trajectory trajectoryReverse;

  private Pose2d pose;

  /**
   * Auto command for testing.
   */
  public TestReverse() {
    drive = DriveTrain.getInstance();

    trajectoryForward = TrajectoryGenerator.generateTrajectory(
        drive.getPoseListFromPathWeaverJson("TurnRight"),
        drive.getTrajectoryConfig(false)
    );

    trajectoryReverse = TrajectoryGenerator.generateTrajectory(
        drive.getPoseListFromPathWeaverJson("ReverseTurnRight"),
        drive.getTrajectoryConfig(true)
    );

    addCommands(
        new InstantCommand(drive::resetOdometry, drive),
        new InstantCommand(drive::zeroEncoder, drive),
        drive.getAutonomousCommandFromTrajectory(trajectoryForward),
        new WaitCommand(2),
        drive.getAutonomousCommandFromTrajectory(trajectoryReverse)
    );
  }
}