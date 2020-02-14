package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;

public class FeederFoward extends CommandBase {

  private Feeder feeder;

  private double motorPower;

  public FeederFoward(double motorPower) {
    feeder = Feeder.getInstance();
    addRequirements(feeder);

    this.motorPower = motorPower;
  }

  @Override
  public void execute() {
    feeder.runFeeder(this.motorPower);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
