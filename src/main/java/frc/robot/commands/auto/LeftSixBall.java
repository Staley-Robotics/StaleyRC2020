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
 * Assumes someone else took our spot. Shoots 3 balls while off centered. Moves to trench to gather
 * 3 more balls. Turns and moves back to face generator and shoots collected balls.
 */
public class LeftSixBall extends LowGearAuto {

  private Vision vision;

  public LeftSixBall() {

    vision = Vision.getInstance();

    Trajectory trajectoryForward = TrajectoryGenerator.generateTrajectory(
        driveTrain.getPoseListFromPathWeaverJson("SpotJackedStart"),
        driveTrain.getTrajectoryConfig(false));

    Trajectory trajectoryForwardContinue = TrajectoryGenerator.generateTrajectory(
        driveTrain.getPoseListFromPathWeaverJson("SpotJackedEnd"),
        driveTrain.getTrajectoryConfig(false));

    addCommands(
        new InstantCommand(driveTrain::resetOdometry, driveTrain),
        new InstantCommand(driveTrain::zeroEncoder, driveTrain),
        new ShootBallsOpenLoop(vision.calculateDistance(vision.getPitch())),
        driveTrain.getAutonomousCommandFromTrajectory(trajectoryForward),
        new ToggleJoint(0.5),
        new RunIntake(defaultIntakePower),
        driveTrain.getAutonomousCommandFromTrajectory(trajectoryForwardContinue),
        new ShootBallsOpenLoop(vision.calculateDistance(vision.getPitch()))
    );
  }
}
