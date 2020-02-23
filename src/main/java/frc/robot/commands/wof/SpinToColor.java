package frc.robot.commands.wof;

import static frc.robot.Constants.WallOfFleshConstants.spinnerPower;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WallOfFlesh;

/**
 * Spins to color. Assumes driver is approaching from near the field wall. Adjust colorOffset for
 * other locations
 */
public class SpinToColor extends CommandBase {

  private WallOfFlesh wallOfFlesh;
  private final int colorOffset = 1;
  private Color[] colors;
  private int findIndex;

  private Color findColor;

  public SpinToColor(Color color) {
    wallOfFlesh = WallOfFlesh.getInstance();
    addRequirements(wallOfFlesh);

    this.findColor = color;
    colors = wallOfFlesh.getColors();
    findIndex = 0;
  }

  @Override
  public void execute() {
    //while not reading color that would make the field's sensor read what we want run motor
    for (int i = 0; i < colors.length; i++) {
      if (wallOfFlesh.getCurrentColor().equals(colors[i])) {
        //avoids index out of bounds error
        findIndex = (i + colorOffset) % colors.length;
        if (!findColor.equals(colors[findIndex])) {
          wallOfFlesh.runWOFSpinner(spinnerPower);
        }
      }
    }
  }

  @Override
  public boolean isFinished() {
    return findColor.equals(colors[findIndex]);
  }

  @Override
  public void end(boolean interrupted) {
    wallOfFlesh.runWOFSpinner(0);
  }
}