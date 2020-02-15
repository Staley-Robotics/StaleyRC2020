package frc.robot.commands.winch;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Winch;

public class RetractWinch extends CommandBase {

  private Winch winch;

  private double motorPower;

  /**
   * Retracts winch at motor power. For example, a new RetractWinch(1) will retract the winch at
   * full speed.
   *
   * @param motorPower power to feed motors.
   */
  public RetractWinch(double motorPower) {
    winch = Winch.getInstance();
    addRequirements(winch);

    this.motorPower = -1 * motorPower;

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
