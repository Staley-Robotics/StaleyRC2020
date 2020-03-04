package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TurnToAngleSimple extends CommandBase {

  private DriveTrain driveTrain;

  private double displacementAngle;
  private double currentAngle;
  private double targetAngle;

  public TurnToAngleSimple(double displacementAngle) {
    driveTrain = DriveTrain.getInstance();
    addRequirements(driveTrain);

    this.displacementAngle = displacementAngle;
  }

  @Override
  public void initialize() {
    currentAngle = driveTrain.getHeading();
    targetAngle = currentAngle - displacementAngle;
  }

  @Override
  public void execute() {
    currentAngle = driveTrain.getHeading();

    if (currentAngle < targetAngle) {
      driveTrain.worldOfTanksDrive(0, 0, -0.5);
    } else if (currentAngle > targetAngle) {
      driveTrain.worldOfTanksDrive(0, 0, 0.5);
    } else {
      driveTrain.worldOfTanksDrive(0, 0, 0);
    }
  }

  @Override
  public boolean isFinished() {
    return Math.abs(currentAngle - targetAngle) < 1;
  }

  @Override
  public void end(boolean interrupted) {
    driveTrain.worldOfTanksDrive(0, 0, 0);
  }
}
