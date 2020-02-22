package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;

public class ShootBallsOpenLoop extends CommandBase {

  private Shooter shooter;
  private Magazine magazine;
  private double distance;

  /**
   * Shoots balls at speed necessary for distance. Make sure to pass in a time to make this work
   *
   * @param distance distance to base shot off of.
   */
  public ShootBallsOpenLoop(double distance) {
    shooter = Shooter.getInstance();
    magazine = Magazine.getInstance();
    addRequirements(shooter, magazine);
    this.distance = distance;
  }

  @Override
  public void initialize() {
    shooter.setFlyWheelSpeed(shooter.calculateSurfaceVelocity(distance));
    magazine.retractHardStop();
  }

  @Override
  public void execute() {
    magazine.runMagazine(0.5);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setFlyWheelSpeed(0);
    magazine.runMagazine(0);
    magazine.extendHardStop();
  }
}
