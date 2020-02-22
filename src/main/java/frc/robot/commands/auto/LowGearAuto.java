package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;

/**
 * Required for autos that start in low gear.
 */
public abstract class LowGearAuto extends SequentialCommandGroup {

  private DriveTrain driveTrain;

  public LowGearAuto() {
    driveTrain = DriveTrain.getInstance();
  }

  @Override
  public void initialize() {
    driveTrain.shiftLow();
    super.initialize();
  }
}
