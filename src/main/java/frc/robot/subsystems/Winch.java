/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.MastConstants.releaseSolenoidPorts;
import static frc.robot.Constants.WinchConstants.leftWinchMotorPort;
import static frc.robot.Constants.WinchConstants.rightWinchMotorPort;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/**
 * Winch subsystem. 2 mini cims, 2 Victor SP. Winch connects from the body of the robot to the
 * climbing payload to pull the robot up during climbing.
 */
public class Winch extends SubsystemBase {

  private static Winch instance;

  private VictorSP leftWinch;
  private VictorSP rightWinch;

  private WinchPistonState winchPistonState;
  private DoubleSolenoid winchPiston;

  /**
   * Winch for climbing.
   */
  private Winch() {
    try {
      leftWinch = new VictorSP(leftWinchMotorPort);
      rightWinch = new VictorSP(rightWinchMotorPort);
    } catch (RuntimeException ex) {
      DriverStation
          .reportError("Error Instantiating Winch Motor Controllers: " + ex.getMessage(), true);
    }
    leftWinch.setInverted(true);
    rightWinch.setInverted(false);
    winchPistonState = WinchPistonState.disengaged;

    try {
      winchPiston = new DoubleSolenoid(releaseSolenoidPorts[0], releaseSolenoidPorts[1]);
    } catch (Exception e) {
      System.out.println("Error instantiating release solenoid");
    }

    retractPiston();
  }

  private enum WinchPistonState {
    engaged,
    disengaged
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
    SmartDashboard.putString("Winch piston", winchPistonState.toString());
  }

  /**
   * Runs winch.
   *
   * @param motorPower power to send to winch motors
   */
  public void runWinch(double motorPower) {
    if (winchPistonState == WinchPistonState.disengaged) {
      leftWinch.set(motorPower);
      rightWinch.set(motorPower);
    }
  }

  /**
   * Toggles winch brake.
   */
  public void togglePiston() {
    if (winchPistonState == WinchPistonState.engaged) {
      retractPiston();
    } else if (winchPistonState == WinchPistonState.disengaged) {
      extendPiston();
    }
  }

  public void retractPiston() {
    winchPiston.set(Value.kReverse);
    winchPistonState = WinchPistonState.disengaged;
  }

  private void extendPiston() {
    winchPiston.set(Value.kForward);
    winchPistonState = WinchPistonState.engaged;
  }
}
