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
 * Immediately shoots pre-loaded balls. Moves to friendly trench to gather 3 balls. Turns and moves
 * back to face the generator and then shoots the 3 collected balls.
 */
public class CentSixBall extends LowGearAuto {

  private Vision vision;

  public CentSixBall() {
    vision = Vision.getInstance();

    Trajectory a_b_ShootThenTrenchIntake = TrajectoryGenerator.generateTrajectory(
        driveTrain.getPoseListFromPathWeaverJson("ShootAndTurn"),
        driveTrain.createTrajectoryConfig(false));

    Trajectory b_c_TurnMoveForwardAndShoot = TrajectoryGenerator.generateTrajectory(
        driveTrain.getPoseListFromPathWeaverJson("GoingForwards"),
        driveTrain.createTrajectoryConfig(false));

    addCommands(
        new InstantCommand(driveTrain::resetOdometry, driveTrain),
        new InstantCommand(driveTrain::zeroEncoder, driveTrain),
        new VisionYawAlign(),
        new ShootBallsOpenLoop(vision.calculateDistance())
            .alongWith(new ToggleJoint()),
        driveTrain.getAutonomousCommandFromTrajectory(a_b_ShootThenTrenchIntake)
            .alongWith(new RunIntake(defaultIntakePower).withTimeout(4)),
        driveTrain.getAutonomousCommandFromTrajectory(b_c_TurnMoveForwardAndShoot),
        new VisionYawAlign(),
        new ShootBallsOpenLoop(vision.calculateDistance())
    );
  }
}