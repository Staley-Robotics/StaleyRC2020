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
 * Starts at the bottom, in line with the enemy's trench. Get 2 balls from enemy trench and then
 * turn to shoot into generator.
 */
public class StealThenShoot extends LowGearAuto {

  private Vision vision;

  public StealThenShoot() {
    vision = Vision.getInstance();

    Trajectory a_b_ToEnemyTrench = TrajectoryGenerator.generateTrajectory(
        driveTrain.getPoseListFromPathWeaverJson("ForwardToTrench"),
        driveTrain.createTrajectoryConfig(false));

    Trajectory b_c_ToGenerator = TrajectoryGenerator.generateTrajectory(
        driveTrain.getPoseListFromPathWeaverJson("ReverseOutOfTrench"),
        driveTrain.createTrajectoryConfig(true));

    addCommands(
        new InstantCommand(driveTrain::resetOdometry, driveTrain),
        new InstantCommand(driveTrain::zeroEncoder, driveTrain),
        driveTrain.getAutonomousCommandFromTrajectory(a_b_ToEnemyTrench)
            .alongWith(new ToggleJoint(), new RunIntake(defaultIntakePower).withTimeout(4)),
        driveTrain.getAutonomousCommandFromTrajectory(b_c_ToGenerator),
        new VisionYawAlign(),
        new ShootBallsOpenLoop(vision.calculateDistance())
    );
  }
}
