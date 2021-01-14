package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;


public class RunShooter extends CommandBase {

  private Shooter shooter;

  private double motorPower;

  public RunShooter(double motorPower) {
    shooter = Shooter.getInstance();
    addRequirements(shooter);

    this.motorPower = motorPower;
  }

  @Override
  public void execute() {
    shooter.runMotors(motorPower);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    shooter.runMotors(0);
  }
}



