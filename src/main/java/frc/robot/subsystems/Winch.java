/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.lMotorFollower1Port;
import static frc.robot.Constants.DriveConstants.lMotorFollower2Port;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Winch extends SubsystemBase {

  private static Winch instance;
  /**
   * Creates a new Winch.
   */
  private WPI_VictorSPX leftWinch;
  private WPI_VictorSPX rightWinch;

  /**
   * Winch for climbing. 2 mini cims powered by 2 Victor SP
   */
  public Winch() {
    leftWinch = new WPI_VictorSPX(lMotorFollower1Port);
    rightWinch = new WPI_VictorSPX(lMotorFollower2Port);
    leftWinch.setInverted(false);
    rightWinch.setInverted(true);
  }

  /**
   * Makes Winch a singleton.
   * @return Wxinch instance
   */
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
