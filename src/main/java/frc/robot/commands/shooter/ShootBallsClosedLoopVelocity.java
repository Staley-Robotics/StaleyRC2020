package frc.robot.commands.shooter;

import static frc.robot.Constants.MagazineConstants.defaultMagazinePower;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class ShootBallsClosedLoopVelocity extends CommandBase {

  private Shooter shooter;
  private Magazine magazine;

  private double distance;
  private double percentSpeedRequired;
  private double goalFlywheelSpeed;

  /**
   * Shoots balls with closed feedback loop at speed necessary for distance. Make sure to pass in a
   * time to make this work
   *
   * @param distance distance to base shot off of.
   */
  public ShootBallsClosedLoopVelocity(double velocity, double percentSpeedRequired) {
    shooter = Shooter.getInstance();
    magazine = Magazine.getInstance();
    addRequirements(shooter, magazine);
    goalFlywheelSpeed = velocity;
    this.percentSpeedRequired = percentSpeedRequired;
  }

  @Override
  public void initialize() {
    //    shooter.setFlyWheelSpeed(shooter.calculateSurfaceVelocity(distance));
    shooter.setFlyWheelSpeed(goalFlywheelSpeed);
  }

  @Override
  public void execute() {

    double flyWheelSpeed = shooter.getFlyWheelSpeedMetersPerSecond();
    shooter.setFlyWheelSpeed(goalFlywheelSpeed);
    double percentage = flyWheelSpeed / goalFlywheelSpeed;
    if (Math.abs(1 - percentage) <= Math.abs(1 - percentSpeedRequired)) {
      System.out.println("Running magazine");
      magazine.retractHardStop();
      magazine.runMagazine(defaultMagazinePower);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setFlyWheelSpeed(0);
    magazine.runMagazine(0);
    //magazine.extendHardStop();
  }
}
