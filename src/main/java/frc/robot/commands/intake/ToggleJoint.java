package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class ToggleJoint extends CommandBase {

  private Intake intake;

  public ToggleJoint() {
    intake = Intake.getInstance();

    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.toggleIntake();
  }

  /**
   * Ends the command.
   */
  @Override
  public boolean isFinished() {
    return true;
  }
}