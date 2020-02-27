package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShootSpeedTest extends CommandBase {

  private Shooter shooter;
  private double speed;

  public ShootSpeedTest(double speed) {
    shooter = Shooter.getInstance();
    addRequirements(shooter);

    this.speed = speed;
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    shooter.setFlyWheelSpeed(speed);
  }

  @Override
  public boolean isFinished() {
return false;
  }
  @Override
  public void end(boolean interrupted) {

  }
}
