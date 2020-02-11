package frc.robot.commands.auto;

import static frc.robot.Constants.DriveConstants.kinematics;
import static frc.robot.Constants.DriveConstants.maxAccelerationMetersPerSecondSquared;
import static frc.robot.Constants.DriveConstants.maxSpeedMetersPerSecond;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;

public class TestReverse {

  private DriveTrain drive;

  private TrajectoryConfig trajectoryConfig;
  private Trajectory trajectory;

  private Pose2d pose;

  /**
   * Currently unfinished, Drives robot in reverse/\.
   */
  public TestReverse() {
    drive = DriveTrain.getInstance();

    trajectoryConfig = new TrajectoryConfig(
        maxSpeedMetersPerSecond,
        maxAccelerationMetersPerSecondSquared)
        .setKinematics(kinematics)
        .setStartVelocity(0.5)
        .setEndVelocity(0.5)
        .setReversed(true);
  }
}