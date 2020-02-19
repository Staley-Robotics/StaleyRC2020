package frc.robot.commands.auto;

import static frc.robot.Constants.IntakeConstants.defaultIntakePower;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.RunIntake;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;

public class Yoink extends SequentialCommandGroup {

  private DriveTrain speed;
  private Intake intake;

  public Yoink() {
    speed = DriveTrain.getInstance();
    intake = Intake.getInstance();

    Trajectory trajectoryForward = TrajectoryGenerator.generateTrajectory(
        speed.getPoseListFromPathWeaverJson("SpotJackedStart"),
        speed.getTrajectoryConfig(false));

    Trajectory trajectoryForwardContinue = TrajectoryGenerator.generateTrajectory(
        speed.getPoseListFromPathWeaverJson("SpotJackedEnd"),
        speed.getTrajectoryConfig(false));

    addCommands(
        new InstantCommand(speed::zeroEncoder, speed),
        //shooter command
        speed.getAutonomousCommandFromTrajectory(trajectoryForward),
        new RunIntake(defaultIntakePower),
        speed.getAutonomousCommandFromTrajectory(trajectoryForwardContinue)
    );
  }
}
