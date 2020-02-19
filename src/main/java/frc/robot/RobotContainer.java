/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static frc.robot.Constants.IntakeConstants.defaultIntakePower;
import static frc.robot.Constants.IntakeConstants.defualtJointPower;
import static frc.robot.Constants.OperatorInputConstants.altControllerPort;
import static frc.robot.Constants.OperatorInputConstants.driveControllerPort;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.intake.ToggleJoint;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private XboxController driveController;
  private XboxController altController;

  private final DriveTrain drive;
  private final Pneumatics pneumatics;
  private final Intake intake;
  private final Vision vision;

  private SendableChooser<Command> autoChooser;


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    drive = DriveTrain.getInstance();
    pneumatics = Pneumatics.getInstance();
    intake = Intake.getInstance();
    vision = Vision.getInstance();

    autoChooser = new SendableChooser<>();

    SmartDashboard.putData("Auto", autoChooser);

    drive.setDefaultCommand(
        new RunCommand(
            () ->
                drive.worldOfTanksDrive(
                    driveController.getTriggerAxis(GenericHID.Hand.kRight),
                    driveController.getTriggerAxis(GenericHID.Hand.kLeft),
                    driveController.getX(GenericHID.Hand.kLeft)),
            drive));

    configureButtonBindings();
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

    /* Alt Controller */

    JoystickButton toggleIntake = new JoystickButton(altController, Button.kX.value);
    toggleIntake.toggleWhenPressed(new RunIntake(defaultIntakePower));

    JoystickButton toggleJointPosition = new JoystickButton(altController, Button.kY.value);
    toggleJointPosition.whenPressed(new ToggleJoint(defualtJointPower).withTimeout(1));

    JoystickButton toggleCompressor = new JoystickButton(altController, Button.kBumperLeft.value);
    toggleCompressor.whenPressed(pneumatics::compressorToggle, pneumatics);

    JoystickButton togglePiston = new JoystickButton(altController, Button.kA.value);
    togglePiston.whenPressed(drive::toggleShift, drive);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous.
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}