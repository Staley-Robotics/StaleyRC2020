package frc.robot.commands.shooter;

import static frc.robot.Constants.ShooterConstants.shooterClosedLoopThreshold;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.TurnToAngle;
import frc.robot.commands.vision.VisionYawAlign;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

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
          //new VisionYawAlign().withTimeout(3),
          new RunShooter(1).withTimeout(0.1),
          new ShootBallsClosedLoop(Vision.getInstance().calculateDistance(),
              shooterClosedLoopThreshold));
    }
  }
}
