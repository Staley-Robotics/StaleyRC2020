package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.shooter.ShootBallsOpenLoop;
import frc.robot.commands.vision.VisionYawAlign;
import frc.robot.subsystems.Vision;

/**
 * Shoots stored balls, then moves off the auto line.
 */
public class ShootThenMoveOff extends LowGearAuto {

  private Vision vision;

  public ShootThenMoveOff() {

    vision = Vision.getInstance();

    Trajectory forwardPastAutoLine = TrajectoryGenerator.generateTrajectory(
        driveTrain.getPoseListFromPathWeaverJson("Forward"),
        driveTrain.getTrajectoryConfig(true));

    addCommands(
        new InstantCommand(driveTrain::resetOdometry, driveTrain),
        new InstantCommand(driveTrain::zeroEncoder, driveTrain),
        new VisionYawAlign(),
        new ShootBallsOpenLoop(vision.calculateDistance(vision.getPitch())),
        driveTrain.getAutonomousCommandFromTrajectory(forwardPastAutoLine)
    );
  }
}
