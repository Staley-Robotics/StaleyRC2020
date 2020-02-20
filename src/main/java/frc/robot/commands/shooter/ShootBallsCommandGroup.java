package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.magazine.RunMagazineBackwards;
import frc.robot.commands.vision.VisionYawAlign;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Vision;

public class ShootBallsCommandGroup extends SequentialCommandGroup {

  private Magazine magazine;
  private Vision vision;

  /**
   * Performs all the necessary steps for shooting a ball.
   */
  public ShootBallsCommandGroup() {

    magazine = Magazine.getInstance();
    vision = Vision.getInstance();

    addCommands(
        new VisionYawAlign(),
        new RunMagazineBackwards(0.5).withTimeout(0.0001),
        new InstantCommand(magazine::retractHardStop, magazine),
        new ShootBalls(vision.calculateDistance(vision.getPitch())).withTimeout(5)
    );
  }
}
