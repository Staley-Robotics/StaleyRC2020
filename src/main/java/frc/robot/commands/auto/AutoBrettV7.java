package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;

/**
 * Shoots three balls, picks up five more, then gets in position for spinning the wheel.
 */
public class AutoBrettV7 extends SequentialCommandGroup {

  private DriveTrain drive;
  private Intake intake;

  public AutoBrettV7() {

    drive = DriveTrain.getInstance();
    intake = Intake.getInstance();

    Trajectory forwardPastAutoLine = TrajectoryGenerator.generateTrajectory(
        drive.getPoseListFromPathWeaverJson("Forward"),
        drive.getTrajectoryConfig(false));

    addCommands(
        new InstantCommand(drive::zeroEncoder, drive),
        drive.getAutonomousCommandFromTrajectory(forwardPastAutoLine)
    );
  }
}
