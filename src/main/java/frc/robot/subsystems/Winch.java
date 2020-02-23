/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.lMotorFollower1Port;
import static frc.robot.Constants.DriveConstants.lMotorFollower2Port;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Winch subsystem. 2 mini cims, 2 Victor SP. Winch connects from the body of the robot to the
 * climbing payload to pull the robot up during climbing.
 */
public class Winch extends SubsystemBase {

  private static Winch instance;
  private VictorSP leftWinch;
  private VictorSP rightWinch;

  /**
   * Winch for climbing.
   */
  private Winch() {
    try {
      leftWinch = new VictorSP(lMotorFollower1Port);
      rightWinch = new VictorSP(lMotorFollower2Port);
    } catch (RuntimeException ex) {
      DriverStation
          .reportError("Error Instantiating Winch Motor Controllers: " + ex.getMessage(), true);
    }
    leftWinch.setInverted(false);
    rightWinch.setInverted(true);
  }

  public static Winch getInstance() {
    if (instance == null) {
      instance = new Winch();
    }
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runWinch(double motorPower) {
    leftWinch.set(motorPower);
    rightWinch.set(motorPower);
  }
}
