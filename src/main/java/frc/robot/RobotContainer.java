/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static frc.robot.Constants.IntakeConstants.defaultIntakePower;
import static frc.robot.Constants.MagazineConstants.defaultMagazinePower;
import static frc.robot.Constants.OperatorInputConstants.altControllerPort;
import static frc.robot.Constants.OperatorInputConstants.driveControllerPort;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.auto.AutoBrettV7;
import frc.robot.commands.auto.CentSixBall;
import frc.robot.commands.auto.LeftSixBall;
import frc.robot.commands.auto.StealThenShoot;
import frc.robot.commands.auto.ShootThenMoveOff;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.intake.ToggleJoint;
import frc.robot.commands.magazine.RunMagazine;
import frc.robot.commands.mast.RunMast;
import frc.robot.commands.shooter.ShootBallsCommandGroup;
import frc.robot.commands.vision.VisionYawAlign;
import frc.robot.commands.winch.RunWinch;
import frc.robot.commands.wof.SpinToColor;
import frc.robot.commands.wof.SpinToCount;
import frc.robot.enums.XboxController.DpadButton;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.WallOfFlesh;

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
  private final WallOfFlesh wallOfFlesh;
  private final Shooter shooter;

  private SendableChooser<Command> autoChooser;


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    drive = DriveTrain.getInstance();
    pneumatics = Pneumatics.getInstance();
    wallOfFlesh = WallOfFlesh.getInstance();
    shooter = Shooter.getInstance();

    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("AutoBrettV7", new AutoBrettV7());
    autoChooser.addOption("Center Six Ball", new CentSixBall());
    autoChooser.addOption("Shoot Then Move", new ShootThenMoveOff());
    autoChooser.addOption("Spot Jacked", new LeftSixBall());
    autoChooser.addOption("Yoink", new StealThenShoot());

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

    JoystickButton runIntake = new JoystickButton(altController, Button.kA.value);
    runIntake.whenHeld(new RunIntake(defaultIntakePower))
        .whenHeld(new RunMagazine(defaultMagazinePower));

    JoystickButton runIntakeBackwards = new JoystickButton(altController, Button.kBack.value);
    runIntakeBackwards.whenHeld(new RunIntake(-defaultIntakePower));

    JoystickButton toggleJointPosition = new JoystickButton(altController, Button.kX.value);
    toggleJointPosition.whenPressed(new ToggleJoint());

    JoystickButton toggleCompressor = new JoystickButton(altController, Button.kY.value);
    toggleCompressor.whenPressed(pneumatics::compressorToggle, pneumatics);

    JoystickButton shoot = new JoystickButton(altController, Button.kB.value);
    shoot.whenPressed(new ShootBallsCommandGroup());

    JoystickButton wofUp = new JoystickButton(altController, DpadButton.kDpadUp.value);
    wofUp.whenPressed(wallOfFlesh::raiseWof, wallOfFlesh);

    JoystickButton wofDown = new JoystickButton(altController, DpadButton.kDpadDown.value);
    wofDown.whenPressed(wallOfFlesh::lowerWof, wallOfFlesh);

    JoystickButton wofSpinNumber = new JoystickButton(altController, DpadButton.kDpadLeft.value);
    wofSpinNumber.whenPressed(new SpinToCount(3.5));

    JoystickButton wofSpinColor = new JoystickButton(altController, DpadButton.kDpadRight.value);
    wofSpinColor.whenPressed(new SpinToColor(Color.kBlue));

    JoystickButton mastUp = new JoystickButton(altController, Button.kBumperLeft.value);
    mastUp.whileHeld(new RunMast(0.5));

    JoystickButton mastDown = new JoystickButton(altController, Button.kBumperRight.value);
    mastDown.whileHeld(new RunMast(-0.5));

    JoystickButton winchExtend = new JoystickButton(altController, Axis.kRightTrigger.value);
    winchExtend.whileHeld(new RunWinch(0.5));

    JoystickButton winchRetract = new JoystickButton(altController, Axis.kLeftTrigger.value);
    winchRetract.whileHeld(new RunWinch(-0.5));

    JoystickButton togglePiston = new JoystickButton(driveController, Button.kX.value);
    togglePiston.whenPressed(drive::toggleShift, drive);

    JoystickButton lineUpShot = new JoystickButton(driveController, Button.kB.value);
    lineUpShot.whenPressed(new VisionYawAlign());
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