package frc.robot.commands.wof;

import static frc.robot.Constants.WallOfFleshConstants.wallOfFleshCircumference;
import static frc.robot.Constants.WallOfFleshConstants.wallOfFleshSpinnerCircumference;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WallOfFlesh;

/**
 * Spins to the color wheel {@link #spinCount} number of rotations.
 */
public class SpinToCount extends CommandBase {

  private WallOfFlesh wallOfFlesh;
  private double spinCount;

  public SpinToCount(double count) {
    wallOfFlesh = WallOfFlesh.getInstance();
    addRequirements(wallOfFlesh);

    this.spinCount = count;
  }

  @Override
  public void execute() {
    wallOfFlesh
        .spinDistance(spinCount * wallOfFleshCircumference / wallOfFleshSpinnerCircumference);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    wallOfFlesh.runWOFSpinner(0);
  }
}