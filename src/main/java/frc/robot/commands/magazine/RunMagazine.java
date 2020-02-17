package frc.robot.commands.magazine;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Magazine;

public class RunMagazine extends CommandBase {

  private Magazine magazine;

  private double motorPower;

  /**
   * Runs magazine at motor power. For example, a new RunMagazine(1) will run the magazine towards
   * the shooter at top speed.
   *
   * @param motorPower power to feed motors.
   */
  public RunMagazine(double motorPower) {
    magazine = Magazine.getInstance();
    addRequirements(magazine);

    this.motorPower = motorPower;
  }

  @Override
  public void execute() {
    magazine.runMagazine(motorPower);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
