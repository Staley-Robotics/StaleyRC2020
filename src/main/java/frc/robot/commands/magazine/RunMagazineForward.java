package frc.robot.commands.magazine;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Magazine;

public class RunMagazineForward extends CommandBase {

  private Magazine magazine;

  private double motorPower;

  /**
   * Runs magazine in forward at motor power. For example, a new RunMagazineForward(1) will run the
   * magazine forward at top speed.
   *
   * @param motorPower power to feed motors.
   */
  public RunMagazineForward(double motorPower) {
    magazine = Magazine.getInstance();
    addRequirements(magazine);

    this.motorPower = motorPower;
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
