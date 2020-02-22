package frc.robot.commands.auto;

import static frc.robot.Constants.IntakeConstants.defaultIntakePower;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.intake.ToggleJoint;
import frc.robot.commands.shooter.ShootBalls;
import frc.robot.commands.shooter.ShootBallsOpenLoop;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

/**
 * Starts at the bottom, in line with the enemy's trench. Get 2 balls from enemy trench and then
 * turn to shoot into generator.
 */
public class RightToEnemyTrenchToShoot extends SequentialCommandGroup {

  private DriveTrain drive;
  private Vision vision;

  public RightToEnemyTrenchToShoot() {
    drive = DriveTrain.getInstance();
    vision = Vision.getInstance();

    Trajectory trajectoryForward = TrajectoryGenerator.generateTrajectory(
        drive.getPoseListFromPathWeaverJson("ForwardToTrench"),
        drive.getTrajectoryConfig(false));

    Trajectory trajectoryForwardContinue = TrajectoryGenerator.generateTrajectory(
        drive.getPoseListFromPathWeaverJson("ReverseOutOfTrench"),
        drive.getTrajectoryConfig(true));

    addCommands(
        new InstantCommand(drive::resetOdometry, drive),
        new InstantCommand(drive::zeroEncoder, drive),
        drive.getAutonomousCommandFromTrajectory(trajectoryForward),
        new ToggleJoint(0.5),
        new RunIntake(defaultIntakePower),
        drive.getAutonomousCommandFromTrajectory(trajectoryForwardContinue),
        new ShootBallsOpenLoop(vision.calculateDistance(vision.getPitch()))
    );
  }
}
