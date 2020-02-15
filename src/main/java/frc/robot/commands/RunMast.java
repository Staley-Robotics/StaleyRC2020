package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Mast;
import frc.robot.subsystems.Winch;


public class RunMast extends CommandBase {

  private Mast mast;

  private double motorPower;

  /**
   * Runs mast at motor power. Positive is extend
   *
   * @param motorPower power to feed motors.
   */
  public RunMast(double motorPower) {
    mast = Mast.getInstance();
    addRequirements(mast);

    this.motorPower = motorPower;
  }

  @Override
  public void execute() {
    mast.runMast(motorPower);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    mast.runMast(0);
  }
}




