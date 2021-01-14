package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;


public class RunIntake extends CommandBase {

  private Intake intake;

  private double motorPower;

  public RunIntake(double motorPower) {
    intake = Intake.getInstance();
    addRequirements(intake);

    this.motorPower = motorPower;
  }

  @Override
  public void execute() {
    intake.runIntake(motorPower);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    intake.runIntake(0);
  }
}



