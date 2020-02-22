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

  /**
   * Whenever end is called, set PivotState to the opposite state and stop motors from moving.
   */
  @Override
  public void end(boolean interrupted) {
    if (intake.getPivotState() == PivotState.up) {
      intake.setPivotState(intake.getPivotState());
    } else if (intake.getPivotState() == PivotState.down) {
      intake.setPivotState(intake.getPivotState());
    }
  }
}