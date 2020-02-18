/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.flyWheelRadius;
import static frc.robot.Constants.ShooterConstants.leftShooterNeoPort;
import static frc.robot.Constants.ShooterConstants.rightShooterNeoPort;
import static frc.robot.Constants.ShooterConstants.shooterD;
import static frc.robot.Constants.ShooterConstants.shooterF;
import static frc.robot.Constants.ShooterConstants.shooterI;
import static frc.robot.Constants.ShooterConstants.shooterP;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Two neos spin a single flywheel shooter at incredibly high speed.
 */
public class Shooter extends SubsystemBase {

  private CANSparkMax leftShooterNeo;
  private CANSparkMax rightShooterNeo;

  private CANPIDController PIDController;

  private CANEncoder shooterEncoder;

  /*These thresholds are used by commands to decide how close the flywhell's velocity needs to be to
  its setpoint before a ball is fed.
   */
  public final double accurateShootVelocityThreshhol = 0.95;
  public final double fastShootVelocityThreshold = 0.8;

  //target height and shooter height in meters
  public final double targetHeight = 5;
  public final double shooterHeight = 4;

  public Shooter() {

    leftShooterNeo = new CANSparkMax(leftShooterNeoPort, MotorType.kBrushless);
    rightShooterNeo = new CANSparkMax(rightShooterNeoPort, MotorType.kBrushless);

    leftShooterNeo.follow(rightShooterNeo, true);

    leftShooterNeo.setIdleMode(IdleMode.kCoast);
    rightShooterNeo.setIdleMode(IdleMode.kCoast);

    shooterEncoder = rightShooterNeo.getEncoder();
    PIDController = rightShooterNeo.getPIDController();
    PIDController.setFeedbackDevice(shooterEncoder);

    updatePIDConstants();
    stop();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Updates the constants for the PID using constants.
   */
  public void updatePIDConstants() {
    PIDController.setOutputRange(0, 1);
    PIDController.setP(shooterP);
    PIDController.setI(shooterI);
    PIDController.setD(shooterD);
    PIDController.setFF(shooterF);
  }

  /**
   * Sets the PID target from surface velocity.
   *
   * @param surfaceVelocity the speed that the surface of the ball goes in meters per second.
   */
  public void setFlyWheelSpeed(double surfaceVelocity) {
    PIDController.setReference(surfaceVelocityToRPM(surfaceVelocity), ControlType.kVelocity);
  }

  /**
   * Calculates the surface velocity from neo RPM.
   *
   * @retrun calculated surface velocity.
   */
  public double getFlyWheelSpeed() {
    return rpmToSurfaceVelocity(rightShooterNeo.getEncoder().getVelocity());
  }

  /**
   * Sets the PID target to zero, letting it coast to a stop.
   */
  public void stop() {
    PIDController.setReference(0, ControlType.kDutyCycle);
  }

  /**
   * Calculates target surface velocity to make out shot at a given distance.
   *
   * @param distance distance from the target in inches.
   */
  public void calculateSurfaceVelocity(double distance) {
    //Lots of commented math I don't want to copy
  }

  /**
   * Calculates surface velocity from RPM.
   *
   * @param rpm revolutions per minute of the NEOS.
   * @return calculated surface velocity in meters per second.
   */
  private double rpmToSurfaceVelocity(double rpm) {
    return (rpm / 60) * 2 * Math.PI * flyWheelRadius;
  }

  /**
   * Calculates RPM from surface velocity.
   *
   * @param surfaceVelocity the speed that the surface of the ball goes in meters per second
   * @return calculated RPM.
   */
  private double surfaceVelocityToRPM(double surfaceVelocity) {
    return surfaceVelocity * 60 / (flyWheelRadius * 2 * Math.PI);
  }
}