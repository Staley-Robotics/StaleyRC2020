package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class ZeroEncoder extends CommandBase {

  DriveTrain drive;

  public ZeroEncoder() {
    drive = DriveTrain.getInstance();
    addRequirements(drive);
  }

  @Override
  public void execute() {
    drive.zeroEncoder();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}