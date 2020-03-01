package frc.robot.commands.auto;

import static frc.robot.Constants.ShooterConstants.shooterOpenLoopThreshold;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.shooter.ShootBallsClosedLoop;
import frc.robot.commands.vision.VisionYawAlign;
import frc.robot.subsystems.Vision;

public class ShootThenMoveOff extends LowGearAuto {

  private Vision vision;

  public ShootThenMoveOff() {
    vision = Vision.getInstance();

    Trajectory a_b_MovePastLine = TrajectoryGenerator.generateTrajectory(
        driveTrain.getPoseListFromPathWeaverJson("Forward"),
        driveTrain.createTrajectoryConfig(true)
    );

    addCommands(
        new InstantCommand(driveTrain::resetOdometry),
        new InstantCommand(driveTrain::zeroEncoder),
        new VisionYawAlign(),
        new ShootBallsClosedLoop(vision.calculateDistance(), shooterOpenLoopThreshold),
        driveTrain.getAutonomousCommandFromTrajectory(a_b_MovePastLine)
    );
  }
}
