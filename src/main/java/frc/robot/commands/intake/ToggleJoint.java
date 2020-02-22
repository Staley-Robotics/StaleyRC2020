package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.PivotState;

public class ToggleJoint extends CommandBase {

  private Intake intake;

  public ToggleJoint() {
    intake = Intake.getInstance();

    addRequirements(intake);
  }

  @Override
  public void initialize() {
    if (intake.getPivotState() == PivotState.up) {
      intake.lowerIntake();
    } else {
      intake.raiseIntake();
    }

    if (intake.getPivotState() == PivotState.up) {
      intake.setPivotState(PivotState.down);
    } else if (intake.getPivotState() == PivotState.down) {
      intake.setPivotState(PivotState.up);
    }
  }

  @Override
  public void execute() {
  }

  /**
   * Ends the command.
   */
  @Override
  public boolean isFinished() {
    return true;
  }
}