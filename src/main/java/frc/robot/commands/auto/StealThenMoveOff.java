package frc.robot.commands.auto;

import static frc.robot.Constants.IntakeConstants.defaultIntakePower;
import static frc.robot.Constants.IntakeConstants.defaultMotorJointPower;
import static frc.robot.Constants.ShooterConstants.shooterOpenLoopThreshold;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.intake.RunIntakeJoint;
import frc.robot.commands.shooter.ShootBallsClosedLoop;
import frc.robot.commands.vision.VisionYawAlign;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision;

public class StealThenMoveOff extends LowGearAuto {

  private Vision vision;
  private Intake intake;

  public StealThenMoveOff() {
    vision = Vision.getInstance();
    intake = Intake.getInstance();

    Trajectory a_b_ToEnemyTrench = TrajectoryGenerator.generateTrajectory(
        driveTrain.getPoseListFromPathWeaverJson("ForwardToTrench"),
        driveTrain.createTrajectoryConfig(false)
    );

    Trajectory b_c_ToGenerator = TrajectoryGenerator.generateTrajectory(
        driveTrain.getPoseListFromPathWeaverJson("ReverseOutOfTrench"),
        driveTrain.createTrajectoryConfig(true)
    );

    addCommands(
        new InstantCommand(driveTrain::resetOdometry, driveTrain),
        new InstantCommand(driveTrain::zeroEncoder, driveTrain),
        driveTrain.getAutonomousCommandFromTrajectory(a_b_ToEnemyTrench).alongWith(
        new InstantCommand(intake::lowerIntake)
            .alongWith(new RunIntake(defaultIntakePower).withTimeout(4))),
        driveTrain.getAutonomousCommandFromTrajectory(b_c_ToGenerator),
        new VisionYawAlign(),
        new ShootBallsClosedLoop(vision.calculateDistance(), shooterOpenLoopThreshold)
    );
  }
}
