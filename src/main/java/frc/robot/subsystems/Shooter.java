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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;

/**
 * Two neos spin a single flywheel shooter at incredibly high speed.
 */
public class Shooter extends SubsystemBase {

  private static Shooter instance;
  private CANSparkMax leftShooterNeo;
  private CANSparkMax rightShooterNeo;

  private CANPIDController PIDController;

  private CANEncoder shooterEncoder;

  /*These thresholds are used by commands to decide how close the flywheel's velocity
  needs to be to its setpoint before a ball is fed.*/

  private double targetSpeed;
  private ArrayList<Double> shootingTargets;

  private void populateShootingTargets() {
    //TODO: populate shooting targets
  }

  private Shooter() {
    shootingTargets = new ArrayList<>();
    populateShootingTargets();
    try {
      leftShooterNeo = new CANSparkMax(leftShooterNeoPort, MotorType.kBrushless);
      rightShooterNeo = new CANSparkMax(rightShooterNeoPort, MotorType.kBrushless);
    } catch (RuntimeException ex) {
      DriverStation
          .reportError("Error Instantiating Shooter Motor Controllers: " + ex.getMessage(), true);
    }
    leftShooterNeo.follow(rightShooterNeo, true);

    leftShooterNeo.setIdleMode(IdleMode.kCoast);
    rightShooterNeo.setInverted(true);
    leftShooterNeo.setInverted(false);
    rightShooterNeo.setIdleMode(IdleMode.kCoast);

    shooterEncoder = rightShooterNeo.getEncoder();
    PIDController = rightShooterNeo.getPIDController();
    PIDController.setFeedbackDevice(shooterEncoder);

    targetSpeed = 0;
    updatePIDConstants();
    stop();
  }

  public static Shooter getInstance() {
    if (instance == null) {
      instance = new Shooter();
    }
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Fly wheel surface speed", getFlyWheelSpeedMetersPerSecond());
    SmartDashboard.putNumber("target", getTargetFlywheelSpeedMetersPerSecond());
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

  public double getTargetFlywheelSpeedMetersPerSecond() {
    return rpmToSurfaceVelocity(targetSpeed);
  }

  /**
   * Calculates the surface velocity from neo RPM.
   *
   * @return calculated surface velocity.
   */
  public double getFlyWheelSpeedMetersPerSecond() {
    return rpmToSurfaceVelocity(rightShooterNeo.getEncoder().getVelocity());
  }

  /**
   * Sets the PID target from surface velocity.
   *
   * @param surfaceVelocity the speed that the surface of the ball goes in meters per second.
   */
  public void setFlyWheelSpeed(double surfaceVelocity) {
    if (surfaceVelocity != -1) {
      targetSpeed = surfaceVelocityToRPM(surfaceVelocity);
      PIDController.setReference(surfaceVelocityToRPM(surfaceVelocity), ControlType.kVelocity);
    }
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
  public double calculateSurfaceVelocity(double distance) {
    if (distance == 0) {
      //m/s bumper on line
      return 21.7;
    }
    int units = (int) distance;
    if (units > shootingTargets.size() - 1) {
      units = shootingTargets.size() - 1;
    }
    return shootingTargets.get(units);
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

  public void runShooter(double triggerAxis) {
    rightShooterNeo.set(triggerAxis);
  }

  public double getTargetSpeed() {
    return targetSpeed;
  }
}