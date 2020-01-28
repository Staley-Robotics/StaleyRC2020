package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;


public class RunIntake extends CommandBase {

  Intake intake;

  private double motorSpeed;

  public RunIntake(double motorSpeed) {
    intake = Intake.getInstance();
    addRequirements(intake);

    this.motorSpeed = motorSpeed;
  }

  @Override
  public void execute() {
    intake.runIntake(this.motorSpeed);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted){
      intake.runIntake(0);
  }
}




