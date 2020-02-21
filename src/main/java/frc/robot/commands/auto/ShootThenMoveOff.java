package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.ShootBallsOpenLoop;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

/**
 * Shoots stored balls, then moves off the auto line.
 */
public class ShootThenMoveOff extends SequentialCommandGroup {

  private DriveTrain drive;
  private Vision vision;

  public ShootThenMoveOff() {

    drive = DriveTrain.getInstance();
    vision = Vision.getInstance();

    Trajectory forwardPastAutoLine = TrajectoryGenerator.generateTrajectory(
        drive.getPoseListFromPathWeaverJson("Forward"),
        drive.getTrajectoryConfig(false));

    addCommands(
        new InstantCommand(drive::resetOdometry, drive),
        new InstantCommand(drive::zeroEncoder, drive),
        new ShootBallsOpenLoop(vision.calculateDistance(vision.getPitch())),
        drive.getAutonomousCommandFromTrajectory(forwardPastAutoLine)
    );
  }
}
