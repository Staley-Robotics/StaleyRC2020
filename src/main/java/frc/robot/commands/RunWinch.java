package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Winch;


public class RunWinch extends CommandBase {

  private Winch winch;

  private double motorPower;

  /**
   * Runs winch at motor power. Positive is extend
   * @param motorPower power to feed motors.
   */
  public RunWinch(double motorPower) {
    winch = Winch.getInstance();
    addRequirements(winch);

    this.motorPower = motorPower;
  }

  @Override
  public void execute() {
    winch.runWinch(motorPower);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    winch.runWinch(0);
  }
}




