/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static frc.robot.Constants.IntakeConstants.defaultIntakePower;
import static frc.robot.Constants.MagazineConstants.defaultMagazinePower;
import static frc.robot.Constants.MastConstants.mastDefaultMotorPower;
import static frc.robot.Constants.OperatorInputConstants.altControllerPort;
import static frc.robot.Constants.OperatorInputConstants.driveControllerPort;
import static frc.robot.Constants.WinchConstants.winchDefaultMotorPower;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drivetrain.TurnToAngle;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.shooter.ShootSpeedTest;
import frc.robot.commands.shooter.TestingShootBallsCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Mast;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.WallOfFlesh;
import frc.robot.subsystems.Winch;
import frc.robot.util.DPadButton;
import frc.robot.util.DPadButton.Direction;

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
  private final Intake intake;
  private final Magazine magazine;
  private final Mast mast;
  private final Pneumatics pneumatics;
  private final Shooter shooter;
  private final Vision vision;
  private final WallOfFlesh wallOfFlesh;
  private final Winch winch;

  private SendableChooser<Command> autoChooser;


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    drive = DriveTrain.getInstance();
    intake = Intake.getInstance();
    magazine = Magazine.getInstance();
    mast = Mast.getInstance();
    pneumatics = Pneumatics.getInstance();
    shooter = Shooter.getInstance();
    vision = Vision.getInstance();
    wallOfFlesh = WallOfFlesh.getInstance();
    winch = Winch.getInstance();

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

    /*magazine.setDefaultCommand(
        new RunCommand(
            () -> magazine.runMagazine(altController.getTriggerAxis(Hand.kRight)), magazine
        )
    );
      */
    // shooter.setDefaultCommand(new RunCommand(
    // () -> shooter.runShooter(altController.getTriggerAxis(GenericHID.Hand.kRight)), shooter));

    // intake.setDefaultCommand(
    // new RunCommand(
    // () -> intake.runIntakeJoint(-altController.getY(GenericHID.Hand.kRight)),
    // intake));

    // mast.setDefaultCommand(new RunCommand(
    // () -> mast.runMastTriggers(altController.getTriggerAxis(
    // GenericHID.Hand.kLeft), altController.getTriggerAxis(GenericHID.Hand.kRight)), mast));
    mast.setDefaultCommand(new RunCommand(
        () -> mast.runMastTriggers(altController.getTriggerAxis(
            GenericHID.Hand.kLeft),
            altController.getTriggerAxis(GenericHID.Hand.kRight)), mast));

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
    // TODO: Undo this comment
    //    JoystickButton runIntake = new JoystickButton(altController, Button.kA.value);
    //    runIntake.whenPressed(new RunIntake(defaultIntakePower))
    //        .whenPressed(new RunMagazine(defaultMagazinePower));

    JoystickButton runIntake = new JoystickButton(altController, Button.kA.value);
    runIntake.whileHeld(new RunIntake(defaultIntakePower))
        .whileHeld(() -> magazine.runMagazine(defaultMagazinePower), magazine)
        .whenReleased(() -> magazine.runMagazine(0));

    JoystickButton runIntakeBackwards = new JoystickButton(altController, Button.kBack.value);
    runIntakeBackwards.whileHeld(new RunIntake(-defaultIntakePower))
        .whenReleased(() -> intake.runIntake(0));

    //    JoystickButton runMagazineBackwardTest =
    //    new JoystickButton(altController, Button.kStart.value);
    //    runMagazineBackwardTest.whileHeld(
    //    () -> magazine.runMagazine(-defaultMagazinePower), magazine)
    //        .whenReleased(() -> magazine.runMagazine(0));
    //    JoystickButton runMagazineForwardTest =
    //    new JoystickButton(altController, Button.kX.value);
    //    runMagazineForwardTest.whileHeld(
    //    () -> magazine.runMagazine(defaultMagazinePower), magazine)
    //        .whenReleased(() -> magazine.runMagazine(0));

    JoystickButton toggleIntakePosition = new JoystickButton(altController, Button.kX.value);
    toggleIntakePosition.whenPressed(intake::toggleIntake, intake);
    //    toggleIntakePosition.whenPressed(new ToggleJoint());

    JoystickButton toggleCompressor = new JoystickButton(altController, Button.kY.value);
    toggleCompressor.whenPressed(pneumatics::compressorToggle, pneumatics);

    JoystickButton shoot = new JoystickButton(altController, Button.kB.value);
    shoot.whileHeld(new TestingShootBallsCommandGroup(true))
        .whenReleased(magazine::extendHardStop, magazine);

    DPadButton wofUp = new DPadButton(altController, Direction.Up);
    wofUp.whenPressed(wallOfFlesh::raiseWof, wallOfFlesh);

    DPadButton wofDown = new DPadButton(altController, Direction.Down);
    wofDown.whenPressed(wallOfFlesh::lowerWof, wallOfFlesh);

    //DPadButton wofSpinNumber = new DPadButton(altController, Direction.Left);
    //wofSpinNumber.whenPressed(new SpinToCount(goalSpinCount));
    DPadButton zeroPID = new DPadButton(altController, Direction.Left);
    zeroPID.whenPressed(shooter::stop, shooter);

    //DPadButton wofSpinColor = new DPadButton(altController, Direction.Right);
    //wofSpinColor.whenPressed(new SpinToColor(wallOfFlesh.getColorTarget()));

    DPadButton setShooterSpeed = new DPadButton(altController, Direction.Right);
    setShooterSpeed.whenHeld(new ShootSpeedTest(20)); // max is 28

    JoystickButton winchExtend = new JoystickButton(altController, Button.kBumperRight.value);
    winchExtend.whileHeld(() -> winch.runWinch(mastDefaultMotorPower), winch)
        .whenReleased(() -> winch.runWinch(0), winch);
    //JoystickButton runMagazine = new JoystickButton(altController, Axis.kRightTrigger.value);
    //runMagazine.whileHeld(() -> magazine.runMagazine(altController.getTriggerAxis(Hand.kRight)));

    JoystickButton winchRetract = new JoystickButton(altController, Button.kBumperLeft.value);
    winchRetract.whileHeld(() -> winch.runWinch(-winchDefaultMotorPower), winch)
        .whenReleased(() -> winch.runWinch(0), winch);

    JoystickButton shiftButton = new JoystickButton(driveController, Button.kX.value);
    shiftButton.whenPressed(drive::toggleShift, drive);

    //    JoystickButton lineUpShot = new JoystickButton(driveController, Button.kB.value);
    //    lineUpShot.whenPressed(new VisionYawAlign());

    JoystickButton lineUpShot = new JoystickButton(driveController, Button.kB.value);
    lineUpShot.whenPressed(new TurnToAngle(90));
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