package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;


public class RunIntakeJoint extends CommandBase {

  private Intake intake;

  private double motorPower;

  public RunIntakeJoint(double motorPower) {
    intake = Intake.getInstance();
    addRequirements(intake);

    this.motorPower = motorPower;
  }

  @Override
  public void execute() {
    intake.runIntakeJoint(motorPower);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    intake.runIntakeJoint(0);
  }
}




