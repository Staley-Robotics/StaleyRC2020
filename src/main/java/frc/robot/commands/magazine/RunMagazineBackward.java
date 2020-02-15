package frc.robot.commands.magazine;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Magazine;

public class RunMagazineBackward extends CommandBase {

  private Magazine magazine;

  private double motorPower;

  /**
   * Runs magazine in reverse at motor power. For example, a new RunMagazineBackward(1) will run the
   * magazine backward at top speed.
   *
   * @param motorPower power to feed motors.
   */
  public RunMagazineBackward(double motorPower) {
    magazine = Magazine.getInstance();
    addRequirements(magazine);

    this.motorPower = -1 * motorPower;
  }

  @Override
  public void execute() {
    magazine.runMagazine(this.motorPower);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
