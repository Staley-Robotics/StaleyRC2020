package frc.robot.commands.auto;

import static frc.robot.Constants.IntakeConstants.defaultIntakePower;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.intake.ToggleJoint;
import frc.robot.commands.shooter.ShootBallsOpenLoop;
import frc.robot.subsystems.Vision;

/**
 * Immediately shoots pre-loaded balls. Moves to friendly trench to gather 3 balls. Turns and moves
 * back to face the generator and then shoots the 3 collected balls.
 */
public class CenteredSixBall extends LowGearAuto {

  private Vision vision;

  public CenteredSixBall() {
    vision = Vision.getInstance();

    Trajectory trajectoryForward = TrajectoryGenerator.generateTrajectory(
        driveTrain.getPoseListFromPathWeaverJson("ShootAndTurn"),
        driveTrain.getTrajectoryConfig(false));

    Trajectory trajectoryForwardContinue = TrajectoryGenerator.generateTrajectory(
        driveTrain.getPoseListFromPathWeaverJson("GoingForwards"),
        driveTrain.getTrajectoryConfig(false));

    addCommands(
        new InstantCommand(driveTrain::resetOdometry, driveTrain),
        new InstantCommand(driveTrain::zeroEncoder, driveTrain),
        new ShootBallsOpenLoop(vision.calculateDistance(vision.getPitch())),
        driveTrain.getAutonomousCommandFromTrajectory(trajectoryForward),
        new ToggleJoint(),
        new RunIntake(defaultIntakePower).withTimeout(10),
        driveTrain.getAutonomousCommandFromTrajectory(trajectoryForwardContinue),
        new ShootBallsOpenLoop(vision.calculateDistance(vision.getPitch()))
    );
  }
}