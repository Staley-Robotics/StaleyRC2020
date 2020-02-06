/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static frc.robot.Constants.DriveConstants.kinematics;
import static frc.robot.Constants.DriveConstants.ramseteB;
import static frc.robot.Constants.DriveConstants.ramseteZ;
import static frc.robot.Constants.IntakeConstants.defaultIntakePower;
import static frc.robot.Constants.IntakeConstants.defualtJointPower;
import static frc.robot.Constants.OperatorInputConstants.altControllerPort;
import static frc.robot.Constants.OperatorInputConstants.driveControllerPort;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.RunIntake;
import frc.robot.commands.ToggleJoint;
import frc.robot.commands.ZeroEncoder;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import java.io.IOException;
import java.nio.file.Paths;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private XboxController driveController;
  private XboxController altController;
  private CommandBase auto;
  private final DriveTrain drive;
  private final Intake intake;
  private Trajectory trajectory;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();

    intake = Intake.getInstance();
    drive = DriveTrain.getInstance();

    auto = null;
    trajectory = null;

    String trajectoryName = "Straight";

    try {
      // Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      // trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      trajectory = loadTrajectory(trajectoryName);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryName, ex.getStackTrace());
    }

    drive.setDefaultCommand(
        new RunCommand(
            () ->
                drive.worldOfTanksDrive(
                    driveController.getTriggerAxis(GenericHID.Hand.kRight),
                    driveController.getTriggerAxis(GenericHID.Hand.kLeft),
                    driveController.getX(GenericHID.Hand.kLeft)),
            drive));
  }

  /**
   * Allows us to choose our Trajectory.
   *
   * @param trajectoryName Name of Trajectory file.
   * @return Trajectory path to be used.
   * @throws IOException Can't find the file in the directory.
   */
  private Trajectory loadTrajectory(String trajectoryName) throws IOException {
    return TrajectoryUtil.fromPathweaverJson(
        Filesystem.getDeployDirectory().toPath().resolve(
            Paths.get("output", trajectoryName + ".wpilib.json")));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    driveController = new XboxController(driveControllerPort);
    altController = new XboxController(altControllerPort);

    // Runs Intake
    JoystickButton toggleIntake = new JoystickButton(altController, Button.kX.value);
    toggleIntake.toggleWhenPressed(new RunIntake(defaultIntakePower));

    JoystickButton toggleJointPosition = new JoystickButton(altController, Button.kY.value);
    toggleJointPosition.whenPressed(new ToggleJoint(defualtJointPower).withTimeout(1));

    JoystickButton zeroEncoder = new JoystickButton(driveController, Button.kA.value);
    zeroEncoder.whenPressed(new ZeroEncoder());
  }

  public void resetOdometry() {
    new InstantCommand(drive::resetOdometry, drive).schedule();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous.
   */
  public Command getAutonomousCommand() {
    return new InstantCommand(drive::zeroEncoder, drive).andThen(drive::zeroYaw, drive)
        .andThen(new RamseteCommand(
            trajectory,
            drive::getPose,
            new RamseteController(ramseteB, ramseteZ),
            kinematics,
            drive::tankDriveVelocity,
            drive))
        .andThen(drive::stopDrive, drive)
        .beforeStarting(drive::zeroEncoder, drive);
  }
}