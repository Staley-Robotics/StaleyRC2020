package frc.robot.commands.auto;

import static frc.robot.Constants.IntakeConstants.defaultIntakePower;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.intake.ToggleJoint;
import frc.robot.commands.shooter.ShootBallsOpenLoop;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

/**
 * Starts at the bottom, in line with the enemy's trench. Get 2 balls from enemy trench and then
 * turn to shoot into generator.
 */
public class RightToEnemyTrenchToShoot extends LowGearAuto {

  private Vision vision;

  public RightToEnemyTrenchToShoot() {
    vision = Vision.getInstance();

    Trajectory trajectoryForward = TrajectoryGenerator.generateTrajectory(
        driveTrain.getPoseListFromPathWeaverJson("ForwardToTrench"),
        driveTrain.getTrajectoryConfig(false));

    Trajectory trajectoryForwardContinue = TrajectoryGenerator.generateTrajectory(
        driveTrain.getPoseListFromPathWeaverJson("ReverseOutOfTrench"),
        driveTrain.getTrajectoryConfig(true));

    addCommands(
        new InstantCommand(drive::resetOdometry, driveTrain),
        new InstantCommand(drive::zeroEncoder, driveTrain),
        drive.getAutonomousCommandFromTrajectory(trajectoryForward),
        new ToggleJoint(),
        new RunIntake(defaultIntakePower),
        driveTrain.getAutonomousCommandFromTrajectory(trajectoryForwardContinue),
        new ShootBallsOpenLoop(vision.calculateDistance(vision.getPitch()))
    );
  }
}
