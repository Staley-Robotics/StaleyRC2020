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
import static frc.robot.Constants.DriveConstants.maxAccelerationMetersPerSecondSquared;
import static frc.robot.Constants.DriveConstants.maxVelocityMetersPerSecond;
import static frc.robot.Constants.DriveConstants.rMotorFollower1Port;
import static frc.robot.Constants.DriveConstants.rMotorFollower2Port;
import static frc.robot.Constants.DriveConstants.rMotorMasterPort;
import static frc.robot.Constants.DriveConstants.ramseteB;
import static frc.robot.Constants.DriveConstants.ramseteZ;
import static frc.robot.Constants.DriveConstants.rotateDeadzone;
import static frc.robot.Constants.DriveConstants.shiftPointMetersPerSecond;
import static frc.robot.Constants.DriveConstants.speedModifier;
import static frc.robot.Constants.DriveConstants.turnSpeedModifier;
import static frc.robot.Constants.DriveConstants.wheelCircumferenceMeters;
import static frc.robot.Constants.PneumaticConstants.shifterPorts;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import javax.json.Json;
import javax.json.JsonArray;
import javax.json.JsonObject;
import javax.json.JsonReader;

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

  private final DifferentialDriveOdometry odometry;
  private Pose2d savedPose;

  private DoubleSolenoid shifter;
  private ShifterState shifterState;

  public enum ShifterState {
    low,
    high
  }

  private DriveTrain() {
    try {
      rightMaster = new WPI_TalonSRX(rMotorMasterPort);
      rightFollower1 = new WPI_VictorSPX(rMotorFollower1Port);
      rightFollower2 = new WPI_VictorSPX(rMotorFollower2Port);

      leftMaster = new WPI_TalonSRX(lMotorMasterPort);
      leftFollower1 = new WPI_VictorSPX(lMotorFollower1Port);
      leftFollower2 = new WPI_VictorSPX(lMotorFollower2Port);

    } catch (RuntimeException ex) {
      DriverStation
          .reportError("Error Instantiating drive motor controllers: " + ex.getMessage(), true);
    }

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
    try {
      gyro = new AHRS();
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error Instantiating Gyro: " + ex.getMessage(), true);
    }

    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    shifter = new DoubleSolenoid(shifterPorts[0], shifterPorts[1]);

    shifterState = ShifterState.low;

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

    SmartDashboard.putString("Drive Shift", getShifterState().toString());
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

    backward = backward * speedModifier;
    forward = forward * speedModifier;

    // Logic for deadzones
    // rotate > rotateDeadZone || rotate < rotateDeadZone
    if (Math.abs(rotate) > rotateDeadzone) {
      rotate = -rotate * turnSpeedModifier;
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
    /* autoshift testing
    if (isLowGearOptimal()) {
      if (shifterState == ShifterState.high) {
        shiftLow();
      }
    } else {
      if (shifterState == ShifterState.low) {
        shiftHigh();
      }
    }
     */
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

  public double getLeftEncoderMetersPerSecondVelocity() {
    return stepsPerDecisecToMetersPerSec(getLeftEncoderVelocity());
  }

  public double getRightEncoderMetersPerSecondVelocity() {
    return stepsPerDecisecToMetersPerSec(getRightEncoderVelocity());
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

  /**
   * Zeros drive encoders.
   */
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
    return Math.IEEEremainder(getYaw(), 360) * -1;
  }

  /**
   * Sets the robot's current position as the origin.
   */
  public void resetOdometry() {
    savedPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    odometry.resetPosition(savedPose, Rotation2d.fromDegrees(getHeading()));
  }

  /* Trajectory */

  /**
   * Gets a TrajectoryConfig.
   *
   * @param isReversed Determines if the bot goes backwards or forwards during a trajectory.
   * @return Trajectory Configuration.
   */
  public TrajectoryConfig createTrajectoryConfig(boolean isReversed) {
    return new TrajectoryConfig(maxVelocityMetersPerSecond, maxAccelerationMetersPerSecondSquared)
        .setKinematics(kinematics)
        .setStartVelocity(0)
        .setEndVelocity(2)
        .setReversed(isReversed);
  }

  /**
   * Replicates data from Pathweaver produced JSON file so that we can input our own Trajectory
   * Configuration.
   *
   * @param trajectoryName Name of Trajectory
   * @return List of Pose2d objects
   */
  public List<Pose2d> getPoseListFromPathWeaverJson(String trajectoryName) {
    ArrayList<Pose2d> poseList = new ArrayList<>();
    InputStream fis;
    JsonReader reader;
    JsonArray wholeFile = null;
    try {
      String trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(
          Paths.get("output", trajectoryName + ".wpilib.json")).toString();

      fis = new FileInputStream(trajectoryPath);

      reader = Json.createReader(fis);

      wholeFile = reader.readArray();

      reader.close();
    } catch (IOException e) {
      System.out.println("CATCH RAN");
      e.printStackTrace();
    }

    for (JsonObject state : wholeFile.getValuesAs(JsonObject.class)) {
      JsonObject pose = state.getJsonObject("pose");
      JsonObject translation = pose.getJsonObject("translation");
      JsonObject rotation = pose.getJsonObject("rotation");

      double x = translation.getJsonNumber("x").doubleValue();
      double y = translation.getJsonNumber("y").doubleValue();
      double radians = rotation.getJsonNumber("radians").doubleValue();

      poseList.add(new Pose2d(x, y, new Rotation2d(radians)));
    }
    return poseList;
  }

  /**
   * Creates a command using trajectory that drives the robot during autonomous.
   *
   * @param trajectory A combination of pose and speed.
   * @return Auto command with given pose.
   */
  public Command getAutonomousCommandFromTrajectory(Trajectory trajectory) {
    return new InstantCommand()
        .andThen(new RamseteCommand(
            trajectory,
            this::getPose,
            new RamseteController(ramseteB, ramseteZ),
            kinematics,
            this::tankDriveVelocity,
            this))
        .andThen(this::stopDrive, this);
  }

  public void shiftLow() {
    shifter.set(Value.kForward);
    shifterState = ShifterState.low;
  }

  public void shiftHigh() {
    shifter.set(Value.kReverse);
    shifterState = ShifterState.high;
  }

  public void runDriveTrain(double power) {
    rightMaster.set(power);
    leftMaster.set(power);
  }

  public ShifterState getShifterState() {
    return shifterState;
  }

  /**
   * Toggles drive shift.
   */
  public void toggleShift() {
    if (shifterState == ShifterState.high) {
      shiftLow();
    } else if (shifterState == ShifterState.low) {
      shiftHigh();
    } else {
      throw new IllegalStateException("it's okay, toggle shift machine broke");
    }
  }

  // There are 2 more options to test here. Creating a shifting threshold,
  // so we don't shift when turning
  // Or, we can automatically shift to high gear when turning
  private boolean isLowGearOptimal() {
    if (Math
        .abs(getLeftEncoderMetersPerSecondVelocity() / getRightEncoderMetersPerSecondVelocity() - 1)
        > (0.2)) {
      return false;
    }

    if (Math.abs(
        (getLeftEncoderMetersPerSecondVelocity() + getRightEncoderMetersPerSecondVelocity()) / 2)
        < shiftPointMetersPerSecond) {
      return true;
    }
    return false;
  }
}