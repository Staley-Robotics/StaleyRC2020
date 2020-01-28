package frc.robot.commands;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.PivotState;
import java.util.logging.ConsoleHandler;

public class RunJoint extends CommandBase {

  private Intake intake;
  StringBuilder message;
  private double motorSpeed;

  public RunJoint(double speed) {
    motorSpeed = speed;

    message = new StringBuilder();

    intake = Intake.getInstance();

    addRequirements(intake);
  }

  @Override
  public void execute() {
    if (intake.getPivotState() == PivotState.down) {
      intake.runJoint(-motorSpeed);
    } else {
      intake.runJoint(motorSpeed);
    }
  }

  @Override
  public boolean isFinished() {
    if (intake.getPivotState() == PivotState.down) {
      return intake.getLimitSwitch();
    } else {
      return false;
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (intake.getPivotState() == PivotState.up) {
      intake.setPivotStateDown();
    } else if (intake.getPivotState() == PivotState.down) {
      intake.setPivotStateUp();
    }
  }

}
