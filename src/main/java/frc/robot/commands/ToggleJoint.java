package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.PivotState;

public class ToggleJoint extends CommandBase {

  private Intake intake;
  private double motorPower;

  public ToggleJoint(double power) {
    motorPower = power;

    intake = Intake.getInstance();

    addRequirements(intake);
  }

  /**
   * If PivotState is down, send negative power. Otherwise, send positive power.
   */
  @Override
  public void execute() {
    if (intake.getPivotState() == PivotState.down) {
      intake.runJoint(-motorPower);
    } else {
      intake.runJoint(motorPower);
    }
  }

  /**
   * IsFinished constantly returns false so the code runs until timeout, end or if the limitswitch is hit.
   */
  @Override
  public boolean isFinished() {
    if (intake.getPivotState() == PivotState.down) {
      return intake.getLimitSwitch();
    } else {
      return false;
    }
  }

  /**
   * Whenever end is called, set PivotState to the opposite state and stop motors from moving.
   */
  @Override
  public void end(boolean interrupted) {
    if (intake.getPivotState() == PivotState.up) {
      intake.runJoint(0);
      intake.setPivotState(intake.getPivotState());
    } else if (intake.getPivotState() == PivotState.down) {
      intake.runJoint(0);
      intake.setPivotState(intake.getPivotState());
    }
  }
}
