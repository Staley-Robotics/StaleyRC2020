package frc.robot.commands.drivetrain;

import static frc.robot.Constants.DriveConstants.turnD;
import static frc.robot.Constants.DriveConstants.turnI;
import static frc.robot.Constants.DriveConstants.turnP;
import static frc.robot.Constants.DriveConstants.turnRateToleranceDegPerS;
import static frc.robot.Constants.DriveConstants.turnToleranceDeg;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveTrain;

/**
 * A command that will turn the robot to the specified angle. Copied from wpilib
 */
public class TurnToAngle extends PIDCommand {

  /**
   * Turns to robot to the specified angle.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param driveTrain         The drive subsystem to use
   */
  public TurnToAngle(double targetAngleDegrees, DriveTrain driveTrain) {
    super(
        new PIDController(turnP, turnI, turnD),
        // Close loop on heading
        driveTrain::getHeading,
        // Set reference to target
        targetAngleDegrees,
        // Pipe output to turn robot
        output -> driveTrain.worldOfTanksDrive(0, 0, output),
        // Require the drive
        driveTrain);

    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(turnToleranceDeg, turnRateToleranceDegPerS);
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atSetpoint();
  }
}
