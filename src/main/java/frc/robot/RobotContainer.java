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
import static frc.robot.Constants.ShooterConstants.autoLineShootSpeed;
import static frc.robot.Constants.ShooterConstants.trenchShootSpeed;
import static frc.robot.Constants.WinchConstants.winchDefaultMotorPower;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.auto.ShootThenMoveOffNoPW;
import frc.robot.commands.auto.ShootThenPushThenBack;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.magazine.RunMagazine;
import frc.robot.commands.shooter.RunShooter;
import frc.robot.commands.shooter.ShootBallsCommandGroupWithSpeed;
import frc.robot.commands.shooter.TestingShootBallsCommandGroup;
import frc.robot.commands.vision.VisionYawAlign;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Mast;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
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
  private final Magazine magazine;
  private final Mast mast;
  private final Pneumatics pneumatics;
  private final Shooter shooter;
  private final Vision vision;
  private final Winch winch;
  private final Intake intake;

  private SendableChooser<Command> autoChooser;


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    drive = DriveTrain.getInstance();
    magazine = Magazine.getInstance();
    mast = Mast.getInstance();
    pneumatics = Pneumatics.getInstance();
    shooter = Shooter.getInstance();
    vision = Vision.getInstance();
    winch = Winch.getInstance();
    intake = Intake.getInstance();

    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("NO PW", new ShootThenMoveOffNoPW());
    autoChooser.addOption("NO PW", new ShootThenMoveOffNoPW());
    autoChooser.addOption("Push", new ShootThenPushThenBack());

    SmartDashboard.putData("Auto", autoChooser);
    //All subsystems will have checks that should be checked before going out.
    //check 1: default commands have been set/not set correctly
    //drive check1
    drive.setDefaultCommand(
        new RunCommand(
            () ->
                drive.worldOfTanksDrive(
                    driveController.getTriggerAxis(GenericHID.Hand.kRight),
                    driveController.getTriggerAxis(GenericHID.Hand.kLeft),
                    driveController.getX(GenericHID.Hand.kLeft)),
            drive));
    //intake check1
    //magazine check1
    //Mast check1
    mast.setDefaultCommand(new RunCommand(
        () -> mast.runMastTriggers(altController.getTriggerAxis(
            GenericHID.Hand.kLeft),
            altController.getTriggerAxis(GenericHID.Hand.kRight)), mast));
    //pneumatics check1
    //shooter check1
    //vision check1
    //winch check1
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

    /* Drive Controller */
    JoystickButton shiftButton = new JoystickButton(driveController, Button.kB.value);
    shiftButton.whenPressed(drive::toggleShift, drive);

    JoystickButton lineUpShot = new JoystickButton(driveController, Button.kX.value);
    lineUpShot.whileHeld(new VisionYawAlign());

    /* Alt Controller */

    JoystickButton shoot = new JoystickButton(altController, Button.kB.value);
    shoot.whileHeld(new TestingShootBallsCommandGroup(true))
        .whenReleased(magazine::extendHardStop, magazine);

    JoystickButton toggleCompressor = new JoystickButton(altController, Button.kY.value);
    toggleCompressor.whenPressed(pneumatics::compressorToggle, pneumatics);

    JoystickButton winchExtend = new JoystickButton(altController, Button.kBumperRight.value);
    winchExtend.whileHeld(() -> winch.runWinch(winchDefaultMotorPower), winch)
        .whenReleased(() -> winch.runWinch(0), winch);

    JoystickButton winchRetract = new JoystickButton(altController, Button.kBumperLeft.value);
    winchRetract.whileHeld(() -> winch.runWinch(-winchDefaultMotorPower), winch)
        .whenReleased(() -> winch.runWinch(0), winch);

    JoystickButton toggleWinch = new JoystickButton(altController, Button.kStart.value);
    toggleWinch.whenPressed(winch::togglePiston, winch);

    JoystickButton runIntakeBackwards = new JoystickButton(altController, Button.kBack.value);
    runIntakeBackwards.whileHeld(new RunIntake(-defaultIntakePower))
        .whileHeld(new RunMagazine(-defaultMagazinePower))
        .whenReleased(() -> intake.runIntake(0)).whenReleased(new RunMagazine(0));

    DPadButton trenchShoot = new DPadButton(altController, Direction.Up);
    trenchShoot.whileHeld(new ShootBallsCommandGroupWithSpeed(trenchShootSpeed)).whenReleased(new InstantCommand(shooter::stop));
    DPadButton autoLineShoot = new DPadButton(altController, Direction.Down);
    autoLineShoot.whileHeld(new ShootBallsCommandGroupWithSpeed(autoLineShootSpeed)).whenReleased(new InstantCommand(shooter::stop));
    DPadButton sendIt = new DPadButton(altController,Direction.Left);
    //30
    sendIt.whileHeld(new ShootBallsCommandGroupWithSpeed(31)).whenReleased(new InstantCommand(shooter::stop));

    JoystickButton runIntake = new JoystickButton(driveController, Button.kA.value);
    runIntake.whileHeld(new RunIntake(defaultIntakePower)).whenReleased(new RunIntake(0));
    DPadButton runShooterBackward = new DPadButton(altController,Direction.Right);
    runShooterBackward.whileHeld(new RunShooter(-0.5)).whenReleased(new InstantCommand(shooter::stop));
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