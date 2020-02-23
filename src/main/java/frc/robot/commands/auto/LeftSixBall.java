package frc.robot.commands.auto;

import static frc.robot.Constants.IntakeConstants.defaultIntakePower;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.intake.ToggleJoint;
import frc.robot.commands.shooter.ShootBallsOpenLoop;
import frc.robot.commands.vision.VisionYawAlign;
import frc.robot.subsystems.Vision;

/**
 * Assumes someone else took our spot. Shoots 3 balls while off centered. Moves to trench to gather
 * 3 more balls. Turns and moves back to face generator and shoots collected balls.
 */
public class LeftSixBall extends LowGearAuto {

  private Vision vision;

  public LeftSixBall() {

    vision = Vision.getInstance();

    Trajectory a_b_ShootThenTrenchIntake = TrajectoryGenerator.generateTrajectory(
        driveTrain.getPoseListFromPathWeaverJson("SpotJackedStart"),
        driveTrain.createTrajectoryConfig(false));

    Trajectory b_c_TurnMoveForwardAndShoot = TrajectoryGenerator.generateTrajectory(
        driveTrain.getPoseListFromPathWeaverJson("SpotJackedEnd"),
        driveTrain.createTrajectoryConfig(false));

    addCommands(
        new InstantCommand(driveTrain::resetOdometry, driveTrain),
        new InstantCommand(driveTrain::zeroEncoder, driveTrain),
        new VisionYawAlign(),
        new ShootBallsOpenLoop(vision.calculateDistance())
            .alongWith(new ToggleJoint()),
        driveTrain.getAutonomousCommandFromTrajectory(a_b_ShootThenTrenchIntake)
            .alongWith(new RunIntake(defaultIntakePower).withTimeout(4)
            ),
        driveTrain.getAutonomousCommandFromTrajectory(b_c_TurnMoveForwardAndShoot),
        new VisionYawAlign(),
        new ShootBallsOpenLoop(vision.calculateDistance())
    );
  }
}
