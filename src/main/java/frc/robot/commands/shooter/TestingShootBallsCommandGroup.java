package frc.robot.commands.shooter;

import static frc.robot.Constants.ShooterConstants.shooterClosedLoopThreshold;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TestingShootBallsCommandGroup extends SequentialCommandGroup {

  /**
   * Performs all the necessary steps for shooting a ball.
   */
  public TestingShootBallsCommandGroup(boolean closedLoop) {
    setup(closedLoop);
  }

  private void setup(boolean closedLoop) {

    if (closedLoop) {
      addCommands(
          new ShootBallsClosedLoop(0, shooterClosedLoopThreshold));
    }
  }
}
