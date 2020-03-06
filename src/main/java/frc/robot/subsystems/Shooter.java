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
import java.util.HashMap;

/**
 * Two neos spin a single flywheel shooter at incredibly high speed.
 */
public class Shooter extends SubsystemBase {

  private static Shooter instance;

  private CANSparkMax leftShooterNeo;
  private CANSparkMax rightShooterNeo;

  private CANEncoder shooterEncoder;
  private CANPIDController PIDController;

  private double targetSpeed;
  private HashMap<Integer, Double> shootingTargets;

  private Shooter() {
    try {
      leftShooterNeo = new CANSparkMax(leftShooterNeoPort, MotorType.kBrushless);
      rightShooterNeo = new CANSparkMax(rightShooterNeoPort, MotorType.kBrushless);
    } catch (RuntimeException ex) {
      DriverStation
          .reportError("Error Instantiating Shooter Motor Controllers: " + ex.getMessage(), true);
    }

    leftShooterNeo.follow(rightShooterNeo, true);
    leftShooterNeo.setIdleMode(IdleMode.kCoast);
    leftShooterNeo.setInverted(false);

    rightShooterNeo.setInverted(true);
    rightShooterNeo.setIdleMode(IdleMode.kCoast);

    shooterEncoder = rightShooterNeo.getEncoder();
    PIDController = rightShooterNeo.getPIDController();
    PIDController.setFeedbackDevice(shooterEncoder);

    targetSpeed = 0;
    updatePIDConstants();
    stop();

    shootingTargets = new HashMap<>();
    populateShootingTargets();
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
   * Sets the PID target to zero, letting it coast to a stop.
   */
  public void stop() {
    PIDController.setReference(0, ControlType.kDutyCycle);
    runMotors(0);
  }

  /**
   * Calculates target surface velocity to make out shot at a given distance.
   *
   * @param distance distance from the target in cm.
   */
  public double calculateSurfaceVelocity(double distance) {
    double tempThing = Math.floor(distance / 25);
    int groupedDistance = (int) tempThing * 25;
    SmartDashboard.putNumber("Group: ", groupedDistance);
    //are you silly? I'm still gonna send it
    if (distance == 0) {
      return 30;
    }
    if (distance < 250) {
      //m/s bumper on line
      return 23;
    }

    if (distance >= 700) {
      return 30;
    }

    return shootingTargets.get(groupedDistance);
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
   * Calculates the surface velocity from neo RPM.
   *
   * @return calculated surface velocity.
   */
  public double getFlyWheelSpeedMetersPerSecond() {
    return rpmToSurfaceVelocity(rightShooterNeo.getEncoder().getVelocity());
  }

  public double getTargetFlywheelSpeedMetersPerSecond() {
    return rpmToSurfaceVelocity(targetSpeed);
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

  private void populateShootingTargets() {
    // {grouped distance from target in cm, speed of the shooter in m/s}
    shootingTargets.put(250, 22.0); //untested
    shootingTargets.put(275, 23.0); //untested
    shootingTargets.put(300, 24.0); //untested
    shootingTargets.put(325, 25.0); // good probably
    shootingTargets.put(350, 24.6); //good on god
    shootingTargets.put(375, 25.00); //tested good
    shootingTargets.put(400, 30.0); //tested good
    shootingTargets.put(425, 30.0); //good on god
    shootingTargets.put(450, 26.3); //tested good
    shootingTargets.put(475, 26.5);
    shootingTargets.put(500, 26.7);
    shootingTargets.put(525, 26.9);
    shootingTargets.put(550, 27.1);
    shootingTargets.put(575, 27.3); //tested good
    shootingTargets.put(600, 27.5); //tested kinda bad but tested
    shootingTargets.put(625, 28.25);//tested good
    shootingTargets.put(650, 28.0);//tested kinda bad but tested
    shootingTargets.put(675, 29.0); //tested good
    shootingTargets.put(700, 30.0);
    shootingTargets.put(725, 30.0);
    shootingTargets.put(750, 30.0);
    shootingTargets.put(775, 30.0);
    shootingTargets.put(800, 30.0); //tested. good
  }

  public void runMotors(double power) {
    rightShooterNeo.set(power);
  }
}