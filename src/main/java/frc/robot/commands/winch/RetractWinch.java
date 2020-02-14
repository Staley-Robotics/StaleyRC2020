package frc.robot.commands.winch;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Winch;

public class RetractWinch extends CommandBase {

  private Winch winch;

  private double motorPower;

  public RetractWinch(double motorPower) {
    winch = Winch.getInstance();
    addRequirements(winch);

    this.motorPower = motorPower;

  }

  @Override
  public void execute() {
    winch.runWinch(-this.motorPower);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
