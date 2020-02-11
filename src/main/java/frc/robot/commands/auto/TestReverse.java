package frc.robot.commands.auto;

import static frc.robot.Constants.DriveConstants.kinematics;
import static frc.robot.Constants.DriveConstants.maxAccelerationMetersPerSecondSquared;
import static frc.robot.Constants.DriveConstants.maxVelocityMetersPerSecond;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveTrain;

public class TestReverse extends SequentialCommandGroup {

  private DriveTrain drive;

  TrajectoryConfig trajectoryConfigForward;
  private TrajectoryConfig trajectoryConfigReverse;

  Trajectory trajectoryForward;
  private Trajectory trajectoryReverse;

  private Pose2d pose;

  /**
   * Auto command for testing.
   */
  public TestReverse() {
    drive = DriveTrain.getInstance();

    trajectoryConfigForward = new TrajectoryConfig(
        maxVelocityMetersPerSecond,
        maxAccelerationMetersPerSecondSquared)
        .setKinematics(kinematics)
        .setStartVelocity(0)
        .setEndVelocity(0)
        .setReversed(false);

    trajectoryConfigReverse = new TrajectoryConfig(
        maxVelocityMetersPerSecond,
        maxAccelerationMetersPerSecondSquared)
        .setReversed(true)
        .setKinematics(kinematics)
        .setStartVelocity(0)
        .setEndVelocity(0);

    trajectoryForward = TrajectoryGenerator.generateTrajectory(
        drive.getPoseListFromPathWeaverJson("TurnRight"),
        trajectoryConfigForward
    );

    trajectoryReverse = TrajectoryGenerator.generateTrajectory(
        drive.getPoseListFromPathWeaverJson("ReverseTurnRight"),
        trajectoryConfigReverse
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