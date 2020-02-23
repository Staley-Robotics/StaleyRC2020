package frc.robot.commands.shooter;

import static frc.robot.Constants.MagazineConstants.defaultMagazinePower;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.magazine.RunMagazine;
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
        new RunMagazine(-defaultMagazinePower).withTimeout(0.005),
        new InstantCommand(magazine::retractHardStop, magazine),
        new ShootBallsOpenLoop(vision.calculateDistance()).withTimeout(5)
    );
  }
}
