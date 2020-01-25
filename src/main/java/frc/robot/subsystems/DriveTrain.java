/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.DriveConstants.*;

public class DriveTrain extends SubsystemBase {

  private static DriveTrain instance;

  private AHRS gyro;

  // Supposedly there are 6 motors for the DriveTrain this time around. 2 Talon, 4 Victor assumption here.
  private WPI_TalonSRX rMaster = new WPI_TalonSRX(rMotorMasterPort);
  private WPI_VictorSPX rFollower1 = new WPI_VictorSPX(rMotorFollower1Port);
  private WPI_VictorSPX rFollower2 = new WPI_VictorSPX(rMotorFollower2Port);

  private final SpeedControllerGroup leftMotors = new SpeedControllerGroup(rMaster, rFollower1, rFollower2);
  
  private WPI_TalonSRX lMaster = new WPI_TalonSRX(lMotorMasterPort);
  private WPI_VictorSPX lFollower1 = new WPI_VictorSPX(lMotorFollower1Port);
  private WPI_VictorSPX lFollower2 = new WPI_VictorSPX(lMotorFollower2Port);

  private final SpeedControllerGroup rightMotors = new SpeedControllerGroup(lMaster, lFollower1, lFollower2);



  // Vroom vroom
  public DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);
  

  /**
   * Creates a new DriveTrain.
   */
  public DriveTrain() {

    rFollower1.follow(rMaster);
    rFollower2.follow(rMaster);

    lFollower1.follow(lMaster);
    lFollower2.follow(lMaster);

  }

  public void tankDrive(double lSpeed, double rSpeed){
    drive.tankDrive(lSpeed, rSpeed);
  }

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



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static DriveTrain getInstance() {
    if (instance == null) {
      instance = new DriveTrain();
    }
    return instance;
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

}
