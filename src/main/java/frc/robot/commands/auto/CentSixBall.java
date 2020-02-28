package frc.robot.commands.auto;

import static frc.robot.Constants.IntakeConstants.defaultIntakePower;
import static frc.robot.Constants.ShooterConstants.shooterOpenLoopThreshold;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.shooter.ShootBallsClosedLoop;
import frc.robot.commands.vision.VisionYawAlign;
import frc.robot.subsystems.Vision;

public class CentSixBall extends LowGearAuto {

  private Vision vision;

  public CentSixBall() {
    vision = Vision.getInstance();

    Trajectory a_b_ShootThenTrenchIntake = TrajectoryGenerator.generateTrajectory(
        driveTrain.getPoseListFromPathWeaverJson("ShootAndTurn"),
        driveTrain.createTrajectoryConfig(false)
    );

    Trajectory b_c_TurnMoveForwardANdShoot = TrajectoryGenerator.generateTrajectory(
        driveTrain.getPoseListFromPathWeaverJson("GoingForwards"),
        driveTrain.createTrajectoryConfig(false)
    );

    addCommands(
        new InstantCommand(driveTrain::resetOdometry),
        new InstantCommand(driveTrain::zeroEncoder),
        new VisionYawAlign(),
        new ShootBallsClosedLoop(vision.calculateDistance(), shooterOpenLoopThreshold)
            .alongWith(new RunIntake(defaultIntakePower).withTimeout(4))
    );
  }
}
