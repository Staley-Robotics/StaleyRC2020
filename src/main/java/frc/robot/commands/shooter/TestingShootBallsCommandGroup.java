package frc.robot.commands.shooter;

import static frc.robot.Constants.ShooterConstants.shooterClosedLoopThreshold;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
          new ShootBallsClosedLoop(3,shooterClosedLoopThreshold));

    }
  }
}
