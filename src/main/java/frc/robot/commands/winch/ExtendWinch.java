package frc.robot.commands.winch;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Winch;

public class ExtendWinch extends CommandBase {

  private Winch winch;

  private double motorPower;

  /**
   * Extends winch at motor power.
   * @param motorPower power to feed motors.
   */
  public ExtendWinch(double motorPower) {
    winch = Winch.getInstance();
    addRequirements(winch);

    this.motorPower = motorPower;
  }

  @Override
  public void execute() {
    winch.runWinch(this.motorPower);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}

