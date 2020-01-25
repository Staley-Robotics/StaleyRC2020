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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {

  private static DriveTrain instance;

  private AHRS gyro;

  public DifferentialDrive drive;
  // 2 Talon, 4 Victor assumption here.
  private WPI_TalonSRX rightMaster = new WPI_TalonSRX(rMotorMasterPort);
  private WPI_VictorSPX rightFollower1 = new WPI_VictorSPX(rMotorFollower1Port);
  private WPI_VictorSPX rightFollower2 = new WPI_VictorSPX(rMotorFollower2Port);
  private final SpeedControllerGroup leftMotors = new SpeedControllerGroup(rightMaster,
      rightFollower1,
      rightFollower2);
  private WPI_TalonSRX leftMaster = new WPI_TalonSRX(lMotorMasterPort);
  private WPI_VictorSPX leftFollower1 = new WPI_VictorSPX(lMotorFollower1Port);
  private WPI_VictorSPX leftFollower2 = new WPI_VictorSPX(lMotorFollower2Port);
  private final SpeedControllerGroup rightMotors = new SpeedControllerGroup(leftMaster,
      leftFollower1,
      leftFollower2);


  public DriveTrain() {

    drive = new DifferentialDrive(leftMotors, rightMotors);

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
   * World drive.
   */
  public void worldOfTanksDrive(double forward, double backward, double rotate) {
    double speedModifier = 1;
    double turnSpeedModifier = 0.85;

    backward = backward * speedModifier;
    forward = forward * speedModifier;
    if (rotate > 0.1 || rotate < 0.1) {
      rotate = rotate * turnSpeedModifier;
    } else {
      rotate = 0;
    }

    if (backward > 0) {
      drive.arcadeDrive(-backward, rotate);
    } else if (forward > 0) {
      drive.arcadeDrive(forward, rotate);
    } else {
      drive.arcadeDrive(0, rotate);
    }
  }

  /**
   * Gets Yaw
   * @return double
   */
  public String getYaw(){
    return Double.toString(gyro.getYaw());
  }

  /**
   * Gets Pitch
   * @return double
   */
  public double getPitch(){
    return Double.valueOf(Double.toString(Double.valueOf(Double.toString(gyro.getPitch()))));
  }

  /**
   * zero yawls
   */
  public void zeroYaw(){
    gyro.zeroYaw();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }



}
