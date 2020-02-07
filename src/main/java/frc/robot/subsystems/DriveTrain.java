/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.countPerRevolution;
import static frc.robot.Constants.DriveConstants.feedForward;
import static frc.robot.Constants.DriveConstants.kD;
import static frc.robot.Constants.DriveConstants.kP;
import static frc.robot.Constants.DriveConstants.kinematics;
import static frc.robot.Constants.DriveConstants.lMotorFollower1Port;
import static frc.robot.Constants.DriveConstants.lMotorFollower2Port;
import static frc.robot.Constants.DriveConstants.lMotorMasterPort;
import static frc.robot.Constants.DriveConstants.rMotorFollower1Port;
import static frc.robot.Constants.DriveConstants.rMotorFollower2Port;
import static frc.robot.Constants.DriveConstants.rMotorMasterPort;
import static frc.robot.Constants.DriveConstants.ramseteB;
import static frc.robot.Constants.DriveConstants.ramseteZ;
import static frc.robot.Constants.DriveConstants.wheelCircumferenceMeters;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.IOException;
import java.nio.file.Paths;
import java.util.Objects;

@SuppressWarnings("FieldCanBeLocal")
public class DriveTrain extends SubsystemBase {

  private static DriveTrain instance;

  private WPI_TalonSRX rightMaster;
  private WPI_VictorSPX rightFollower1;
  private WPI_VictorSPX rightFollower2;

  private WPI_TalonSRX leftMaster;
  private WPI_VictorSPX leftFollower1;
  private WPI_VictorSPX leftFollower2;

  private DifferentialDrive drive;

  private AHRS gyro;
  private boolean isGyroInverted;

  private final DifferentialDriveOdometry odometry;
  private Pose2d savedPose;

  public DriveTrain() {
    rightMaster = new WPI_TalonSRX(rMotorMasterPort);
    rightFollower1 = new WPI_VictorSPX(rMotorFollower1Port);
    rightFollower2 = new WPI_VictorSPX(rMotorFollower2Port);

    leftMaster = new WPI_TalonSRX(lMotorMasterPort);
    leftFollower1 = new WPI_VictorSPX(lMotorFollower1Port);
    leftFollower2 = new WPI_VictorSPX(lMotorFollower2Port);

    TalonSRXConfiguration talonConfig = new TalonSRXConfiguration();
    talonConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;
    talonConfig.slot0.kP = kP;
    talonConfig.neutralDeadband = 0.0;
    talonConfig.slot0.kI = 0.0;
    talonConfig.slot0.kD = kD;
    talonConfig.slot0.integralZone = 400;
    talonConfig.slot0.closedLoopPeakOutput = 1.0;

    rightMaster.configAllSettings(talonConfig);
    leftMaster.configAllSettings(talonConfig);

    rightMaster.setSensorPhase(false);
    leftMaster.setSensorPhase(false);

    rightMaster.setInverted(false);
    rightFollower1.setInverted(false);
    rightFollower2.setInverted(false);

    leftMaster.setInverted(true);
    leftFollower1.setInverted(true);
    leftFollower2.setInverted(true);

    rightFollower1.follow(rightMaster);
    rightFollower2.follow(rightMaster);

    leftFollower1.follow(leftMaster);
    leftFollower2.follow(leftMaster);

    drive = new DifferentialDrive(leftMaster, rightMaster);
    drive.setSafetyEnabled(false);
    drive.setRightSideInverted(false);

    gyro = new AHRS();
    isGyroInverted = true;

    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    zeroEncoder();
  }

  @Override
  public void periodic() {
    odometry.update(Rotation2d.fromDegrees(getHeading()), stepsToMeters(getLeftEncoderPosition()),
        stepsToMeters(getRightEncoderPosition()));

    SmartDashboard.putNumber("Gyro Yaw", getYaw());
    SmartDashboard.putNumber("Gyro Pitch", getPitch());

    SmartDashboard.putNumber("LeftEncoder(m): ", stepsToMeters(getLeftEncoderPosition()));
    SmartDashboard.putNumber("RightEncoder(m): ", stepsToMeters(getRightEncoderPosition()));

    SmartDashboard.putNumber("Heading: ", getHeading());
  }

  /**
   * Makes DriveTrain a singleton.
   */
  public static DriveTrain getInstance() {
    if (instance == null) {
      instance = new DriveTrain();
    }
    return instance;
  }

  /* Drive Code */

  /**
   * Used in teleop as right trigger = forward power, left trigger = backward power, left stick
   * x-axis = rotate power.
   */
  public void worldOfTanksDrive(double forward, double backward, double rotate) {
    double speedModifier = 1;
    double turnSpeedModifier = 0.85;

    backward = backward * speedModifier;
    forward = forward * speedModifier;

    // Deadzones for rotate.
    if (rotate > 0.1 || rotate < 0.1) {
      rotate = rotate * turnSpeedModifier;
    } else {
      rotate = 0;
    }

    // Actual drive logic to move bot.
    if (backward > 0) {
      drive.arcadeDrive(-backward, rotate);
    } else if (forward > 0) {
      drive.arcadeDrive(forward, rotate);
    } else {
      drive.arcadeDrive(0, rotate);
    }
  }

  /**
   * Takes leftVelocity and rightVelocity to accurately move in auto.
   *
   * @param leftVelocity  Motor's left Velocity
   * @param rightVelocity Motor's Right Velocity
   */
  public void tankDriveVelocity(double leftVelocity, double rightVelocity) {
    var leftAccel =
        (leftVelocity - stepsPerDecisecToMetersPerSec(getLeftEncoderVelocity())) / .2;
    var rightAccel =
        (rightVelocity - stepsPerDecisecToMetersPerSec(getRightEncoderVelocity())) / .2;

    var leftFeedForwardVolts = feedForward.calculate(leftVelocity, leftAccel);
    var rightFeedForwardVolts = feedForward.calculate(rightVelocity, rightAccel);

    leftMaster.set(
        ControlMode.Velocity,
        metersPerSecToStepsPerDecisec(leftVelocity),
        DemandType.ArbitraryFeedForward,
        leftFeedForwardVolts / 12);
    rightMaster.set(
        ControlMode.Velocity,
        metersPerSecToStepsPerDecisec(rightVelocity),
        DemandType.ArbitraryFeedForward,
        rightFeedForwardVolts / 12);

    drive.feed();
  }
  //    Invert all motors, encoders, gyro
  //    public void invertDrive(){
  //      rightMaster.setSensorPhase(true);
  //      leftMaster.setSensorPhase(true);
  //
  //      drive.setRightSideInverted(false);
  //
  //      leftMaster.setInverted(true);
  //      leftFollower1.setInverted(true);
  //      leftFollower1.setInverted(true);
  //
  //      // Gyro was already inverted previously, we set it false here.
  //      setGyroInverted(false);
  //    }

  public void stopDrive() {
    drive.arcadeDrive(0, 0);
  }

  /* Gyro */

  /**
   * Gets Yaw. Yaw is the angle which the robot turns.
   *
   * @return double
   */
  public double getYaw() {
    return gyro.getYaw();
  }

  /**
   * Gets Pitch. Pitch is the angle which the robot is oriented among the z axis.
   *
   * @return double
   */
  public double getPitch() {
    return gyro.getPitch();
  }

  public void zeroYaw() {
    gyro.zeroYaw();
  }

  public void setGyroInverted(boolean inversion){
    isGyroInverted = inversion;
  }

  /* Encoder */

  public int getLeftEncoderPosition() {
    return leftMaster.getSelectedSensorPosition(0);
  }

  public int getRightEncoderPosition() {
    return rightMaster.getSelectedSensorPosition(0);
  }

  public int getLeftEncoderVelocity() {
    return leftMaster.getSelectedSensorVelocity();
  }

  public int getRightEncoderVelocity() {
    return rightMaster.getSelectedSensorVelocity();
  }

  /**
   * Converts encoder values to meters.
   *
   * @param steps Encoder values.
   * @return Converted value.
   */
  public static double stepsToMeters(int steps) {
    return (wheelCircumferenceMeters / countPerRevolution) * steps;
  }

  /**
   * Converts encoder velocity to meters per second.
   *
   * @param stepsPerDecisec Encoder readings in Deciseconds.
   * @return Converted value.
   */
  public static double stepsPerDecisecToMetersPerSec(int stepsPerDecisec) {
    return stepsToMeters(stepsPerDecisec * 10);
  }

  /**
   * Converts meters to standard encoder values.
   *
   * @param meters Meters travelled.
   * @return Converted value.
   */
  public static double metersToSteps(double meters) {
    return (meters / wheelCircumferenceMeters) * countPerRevolution;
  }

  /**
   * Conversion back to Deciseconds.
   *
   * @param metersPerSec Encoder readings in seconds.
   * @return Converted value.
   */
  public static double metersPerSecToStepsPerDecisec(double metersPerSec) {
    return metersToSteps(metersPerSec) * .1d;
  }

  public void zeroEncoder() {
    rightMaster.setSelectedSensorPosition(0);
    leftMaster.setSelectedSensorPosition(0);
    System.out.println("Encoders have been zeroed");
  }

  /* Odometry */

  /**
   * See {@Pose2d}.
   *
   * @return Current Pose of the robot.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Ben has acquired genius tier.
   *
   * @return Converts Yaw to 180 to -180.
   */
  public double getHeading() {
    return Math.IEEEremainder(getYaw(), 360) * (isGyroInverted ? -1 : 1);
  }

  /**
   * Sets the robot's current position as the origin.
   */
  public void resetOdometry() {
    zeroEncoder();
    savedPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    odometry.resetPosition(savedPose, Rotation2d.fromDegrees(getHeading()));
  }

  /* Trajectory */

  /**
   * Loads trajectory from given name.
   *
   * @param trajectoryName Name of Trajectory file.
   * @return Trajectory path to be used.
   */
  private Trajectory loadTrajectory(String trajectoryName) {
    try {
      return TrajectoryUtil.fromPathweaverJson(
          Filesystem.getDeployDirectory().toPath().resolve(
              Paths.get("output", trajectoryName + ".wpilib.json")));
    } catch (IOException e) {
      DriverStation.reportError(e.toString(), false);
      return null;
    }
  }

  /**
   * Creates Trajectory Command from trajectory file name.
   *
   * @param trajectoryName Name of trajectory file.
   * @return Auto Command with given trajectory.
   */
  public Command getAutonomousCommand(String trajectoryName) {
    return new InstantCommand(this::zeroEncoder, this)
        .andThen(this::zeroYaw, this)
        .andThen(new RamseteCommand(
            Objects.requireNonNull(loadTrajectory(trajectoryName)),
            this::getPose,
            new RamseteController(ramseteB, ramseteZ),
            kinematics,
            this::tankDriveVelocity,
            this))
        .andThen(this::stopDrive, this);
  }
}