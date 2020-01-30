/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.MastConstants.mastMotorPort;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TelescopicMast extends SubsystemBase {

  private static Intake instance;

  private VictorSP mastMotor;

  public enum MastState {
    expanded,
    retracted
  }

  public TelescopicMast() {
    mastMotor = new VictorSP(mastMotorPort);
  }

  /**
   * You're mom.
   * @return mom.
   */
  public static Intake getInstance() {
    if (instance == null) {
      instance = new Intake();
    }
    return instance;
  }

  public void runMast(double power) {
    mastMotor.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
