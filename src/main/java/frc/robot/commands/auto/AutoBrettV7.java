package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveTrain;

public class AutoBrettV7 extends SequentialCommandGroup {

  private DriveTrain drive;

  /**
   * Drives Forward 1.5 meters, waits 3 seconds and travels another 1.5 meters.
   */
  public AutoBrettV7() {
    drive = DriveTrain.getInstance();

    addCommands(
        new InstantCommand(drive::resetOdometry, drive),
        new InstantCommand(drive::zeroEncoder, drive),
        drive.getAutonomousCommand("TurnRight"),
        new WaitCommand(3),
        drive.getAutonomousCommand("ForwardAfterTurn")
    );
  }
}
