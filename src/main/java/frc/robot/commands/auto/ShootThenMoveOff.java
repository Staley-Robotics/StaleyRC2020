package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;

/**
 * Shoots stored balls, then moves off the auto line.
 */
public class ShootThenMoveOff extends SequentialCommandGroup {

  private DriveTrain drive;

  public ShootThenMoveOff(){

    addCommands(

    );
  }
}
