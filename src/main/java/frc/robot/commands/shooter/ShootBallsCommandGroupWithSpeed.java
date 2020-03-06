package frc.robot.commands.shooter;

import static frc.robot.Constants.ShooterConstants.shooterClosedLoopThreshold;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Vision;

public class ShootBallsCommandGroupWithSpeed extends SequentialCommandGroup {

  /**
   * Performs all the necessary steps for shooting a ball.
   */
  public ShootBallsCommandGroupWithSpeed(double surfaceVelocity) {
    setup(surfaceVelocity);
  }

  private void setup(double surfaceVelocity) {
    addCommands(
        //new VisionYawAlign().withTimeout(3),
        new RunShooter(1).withTimeout(0.1),
        new ShootBallsClosedLoopVelocity(surfaceVelocity,
            shooterClosedLoopThreshold));
  }
}

