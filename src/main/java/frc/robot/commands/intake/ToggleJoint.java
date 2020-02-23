package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.JointState;

public class ToggleJoint extends CommandBase {

  private Intake intake;

  public ToggleJoint() {
    intake = Intake.getInstance();

    addRequirements(intake);
  }

  @Override
  public void initialize() {
    if (intake.getJointState() == JointState.up) {
      intake.lowerIntake();
      intake.setJointState(JointState.down);
    } else if (!intake.getLimitSwitch()) {
      intake.raiseIntake();
      intake.setJointState(JointState.up);
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