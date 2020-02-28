package frc.robot.commands.shooter;

import static frc.robot.Constants.MagazineConstants.defaultMagazinePower;
import static frc.robot.Constants.ShooterConstants.shooterClosedLoopThreshold;
import static frc.robot.Constants.ShooterConstants.shooterOpenLoopThreshold;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.magazine.RunMagazine;
import frc.robot.commands.vision.VisionYawAlign;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Vision;

public class TestingShootBallsCommandGroup extends SequentialCommandGroup {

  private Magazine magazine;
  private Vision vision;

  /**
   * Performs all the necessary steps for shooting a ball.
   */
  public TestingShootBallsCommandGroup(boolean closedLoop) {
    setup(closedLoop);
  }

  private void setup(boolean closedLoop) {
    magazine = Magazine.getInstance();
    if (closedLoop) {
      addCommands(
          new RunMagazine(-defaultMagazinePower).withTimeout(0.005),
          new ShootBallsClosedLoop(1,
              shooterClosedLoopThreshold).withTimeout(5)
      );
    }
  }
}
