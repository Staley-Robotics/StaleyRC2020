/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.lMotorFollower1Port;
import static frc.robot.Constants.DriveConstants.lMotorFollower2Port;
import static frc.robot.Constants.DriveConstants.lMotorMasterPort;
import static frc.robot.Constants.DriveConstants.rMotorFollower1Port;
import static frc.robot.Constants.DriveConstants.rMotorFollower2Port;
import static frc.robot.Constants.DriveConstants.rMotorMasterPort;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {

  private static DriveTrain instance;
  public DifferentialDrive drive;
  private AHRS gyro;

  private WPI_TalonSRX rightMaster;
  private WPI_VictorSPX rightFollower1;
  private WPI_VictorSPX rightFollower2;

  private WPI_TalonSRX leftMaster;
  private WPI_VictorSPX leftFollower1;
  private WPI_VictorSPX leftFollower2;

  public DriveTrain() {
    rightMaster = new WPI_TalonSRX(rMotorMasterPort);
    rightFollower1 = new WPI_VictorSPX(rMotorFollower1Port);
    rightFollower2 = new WPI_VictorSPX(rMotorFollower2Port);
    final SpeedControllerGroup leftMotors = new SpeedControllerGroup(rightMaster, rightFollower1,
        rightFollower2);

    leftMaster = new WPI_TalonSRX(lMotorMasterPort);
    leftFollower1 = new WPI_VictorSPX(lMotorFollower1Port);
    leftFollower2 = new WPI_VictorSPX(lMotorFollower2Port);
    final SpeedControllerGroup rightMotors = new SpeedControllerGroup(leftMaster, leftFollower1,
        leftFollower2);

    gyro = new AHRS();

    drive = new DifferentialDrive(leftMotors, rightMotors);
    drive.setSafetyEnabled(false);

    rightFollower1.follow(rightMaster);
    rightFollower2.follow(rightMaster);

    leftFollower1.follow(leftMaster);
    leftFollower2.follow(leftMaster);
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

  public void tankDrive(double leftSpeed, double rightSpeed) {
    drive.tankDrive(leftSpeed, rightSpeed);
  }

  /**
   * Takes Forward, backward and rotate power to control movement of robot with 3 different inputs.
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

  public void zeroYaw() {
    gyro.zeroYaw();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Gyro Yaw", getYaw());
    SmartDashboard.putNumber("Gyro Pitch", getPitch());
  }

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
}
