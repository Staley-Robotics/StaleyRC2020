package frc.robot.commands.vision;

import static frc.robot.Constants.VisionConstants.cameraDegreeError;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.TurnToAngle;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

/**
 * Turns until we are pointed at our vision target.
 */
public class VisionYawAlign extends SequentialCommandGroup {

  private DriveTrain driveTrain;
  private Vision vision;
  private double angleToTurn;

  /**
   * Uses a PID loop with a setpoint at the yaw we get from vision + the current yaw.
   */
  public VisionYawAlign() {
    driveTrain = DriveTrain.getInstance();
    vision = Vision.getInstance();
  }

  @Override
  public void initialize() {

    //Converts Yaw to 180 to -180.
    angleToTurn = Math.IEEEremainder(-(vision.getYaw() + cameraDegreeError) + driveTrain.getHeading(), 360) * -1;

    addCommands(
        new TurnToAngle(angleToTurn)
    );
    super.initialize();
  }
}
