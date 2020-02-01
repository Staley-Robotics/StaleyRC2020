/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static frc.robot.Constants.DriveConstants.kP;
import static frc.robot.Constants.DriveConstants.kS;
import static frc.robot.Constants.DriveConstants.kV;
import static frc.robot.Constants.DriveConstants.kA;
import static frc.robot.Constants.DriveConstants.kKinematics;
import static frc.robot.Constants.DriveConstants.kMaxAccelerationMetersPerSecondSquared;
import static frc.robot.Constants.DriveConstants.kMaxSpeedMetersPerSecond;
import static frc.robot.Constants.DriveConstants.kRamseteZ;
import static frc.robot.Constants.DriveConstants.kRamseteB;
import static frc.robot.Constants.IntakeConstants.defaultIntakePower;
import static frc.robot.Constants.IntakeConstants.defualtJointPower;
import static frc.robot.Constants.OperatorInputConstants.altControllerPort;
import static frc.robot.Constants.OperatorInputConstants.driveControllerPort;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.RunIntake;
import frc.robot.commands.ToggleJoint;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;


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

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();

    intake = Intake.getInstance();
    drive = DriveTrain.getInstance();

    auto = null;

    drive.setDefaultCommand(
        new RunCommand(
            () ->
                drive.worldOfTanksDrive(
                    driveController.getTriggerAxis(GenericHID.Hand.kRight),
                    driveController.getTriggerAxis(GenericHID.Hand.kLeft),
                    driveController.getX(GenericHID.Hand.kLeft)),
            drive));
  }

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

    JoystickButton toggleIntake = new JoystickButton(altController, Button.kX.value);
    toggleIntake.toggleWhenPressed(new RunIntake(defaultIntakePower));

    JoystickButton toggleJointPosition = new JoystickButton(altController, Button.kY.value);
    toggleJointPosition.whenPressed(new ToggleJoint(defualtJointPower).withTimeout(1));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(kS,
                kV,
                kA),
            kKinematics, 8);
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(kMaxSpeedMetersPerSecond,
            kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(kKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // Straight
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
            new Translation2d(0.5, 0)
        ),
        new Pose2d(1, 0, new Rotation2d(0)),
        config
    );

//    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
//        // Start at the origin facing the +X direction
//        new Pose2d(0, 0, new Rotation2d(0)),
//        // Pass through these two interior waypoints, making an 's' curve path
//        List.of(
//            new Translation2d(1, 1),
//            new Translation2d(2, -1)
//        ),
//        // End 3 meters straight ahead of where we started, facing forward
//        new Pose2d(3, 0, new Rotation2d(0)),
//        // Pass config
//        config
//    );

//    Trajectory trajectory = null;
//    try {
//      trajectory = loadTrajectory("Straight");
//    } catch (IOException e) {
//      e.printStackTrace();
//    }

//    String trajectoryJSON = "output/Straight.wpilib.json";
//
//    Trajectory trajectory = null;
//
//    try {
//      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
//      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
//    } catch (IOException ex) {
//      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
//    }

    return new InstantCommand(drive::zeroEncoder, drive).andThen(new RamseteCommand(
        trajectory,
        drive::getPose,
        new RamseteController(kRamseteB, kRamseteZ),
        kKinematics,
        drive::tankDriveVelocity,
        drive))
        .andThen(drive::stopDrive, drive)
        .beforeStarting(drive::zeroEncoder, drive);
  }
}
