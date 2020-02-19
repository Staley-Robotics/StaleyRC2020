package frc.robot.commands.auto;

import static frc.robot.Constants.IntakeConstants.defaultIntakePower;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.RunIntake;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;

public class AutoTwo extends SequentialCommandGroup {

  private DriveTrain drive;
  private Intake intake;

  public AutoTwo() {

    drive = DriveTrain.getInstance();
    intake = Intake.getInstance();

    Trajectory trajectoryForward = TrajectoryGenerator.generateTrajectory(
        drive.getPoseListFromPathWeaverJson("ShootAndTurn"),
        drive.getTrajectoryConfig(false));

    Trajectory trajectoryForwardContinue = TrajectoryGenerator.generateTrajectory(
        drive.getPoseListFromPathWeaverJson("GoingForwards"),
        drive.getTrajectoryConfig(false));

    addCommands(
        new InstantCommand(drive::resetOdometry, drive),
        new InstantCommand(drive::zeroEncoder, drive),
        //needs a shooter command here
        drive.getAutonomousCommandFromTrajectory(trajectoryForward),
        new RunIntake(defaultIntakePower).withTimeout(10),
        drive.getAutonomousCommandFromTrajectory(trajectoryForwardContinue)
        //needs a shooter command here
    );
  }
}