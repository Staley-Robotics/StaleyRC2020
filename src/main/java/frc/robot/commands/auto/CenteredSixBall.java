package frc.robot.commands.auto;

import static frc.robot.Constants.IntakeConstants.defaultIntakePower;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.intake.ToggleJoint;
import frc.robot.commands.shooter.ShootBalls;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

/**
 * Immediately shoots pre-loaded balls. Moves to friendly trench to gather 3 balls. Turns and moves
 * back to face the generator and then shoots the 3 collected balls.
 */
public class CenteredSixBall extends SequentialCommandGroup {

  private DriveTrain drive;
  private Vision vision;

  public CenteredSixBall() {

    drive = DriveTrain.getInstance();
    vision = Vision.getInstance();

    Trajectory trajectoryForward = TrajectoryGenerator.generateTrajectory(
        drive.getPoseListFromPathWeaverJson("ShootAndTurn"),
        drive.getTrajectoryConfig(false));

    Trajectory trajectoryForwardContinue = TrajectoryGenerator.generateTrajectory(
        drive.getPoseListFromPathWeaverJson("GoingForwards"),
        drive.getTrajectoryConfig(false));

    addCommands(
        new InstantCommand(drive::resetOdometry, drive),
        new InstantCommand(drive::zeroEncoder, drive),
        new ShootBalls(vision.calculateDistance(vision.getPitch())),
        drive.getAutonomousCommandFromTrajectory(trajectoryForward),
        new ToggleJoint(0.5),
        new RunIntake(defaultIntakePower).withTimeout(10),
        drive.getAutonomousCommandFromTrajectory(trajectoryForwardContinue),
        new ShootBalls(vision.calculateDistance(vision.getPitch()))
    );
  }
}