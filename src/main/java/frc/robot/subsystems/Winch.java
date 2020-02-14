/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.WinchConstants.leftWinchPort;
import static frc.robot.Constants.WinchConstants.rightWinchPort;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Winch extends SubsystemBase {

  private static Winch instance;
  private WPI_VictorSPX leftWinch;
  private WPI_VictorSPX rightWinch;

  public Winch() {

    leftWinch = new WPI_VictorSPX(leftWinchPort);
    rightWinch = new WPI_VictorSPX(rightWinchPort);
    leftWinch.setInverted(false);
    rightWinch.setInverted(true);
  }

  /**
   * MAkes Winch a singleton.
   * @return
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

  public void runWinch(double power) {
    leftWinch.set(power);
    rightWinch.set(power);
  }
}
