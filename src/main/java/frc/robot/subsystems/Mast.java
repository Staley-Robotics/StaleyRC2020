/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.MastConstants.mastDeadzone;
import static frc.robot.Constants.MastConstants.mastMotorPort;
import static frc.robot.Constants.MastConstants.potPort;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Mast Subsystem. 1 VictorSP, 1 mini cim. Telescoping mast carries the climber's payload up to the
 * bar. Then we winch ourselves up.
 */
public class Mast extends SubsystemBase {

  private static Mast instance;

  private VictorSP mastMotor;
  private AnalogPotentiometer pot;

  private Mast() {
    mastMotor = new VictorSP(mastMotorPort);
    pot = new AnalogPotentiometer(potPort);
  }

  public static Mast getInstance() {
    if (instance == null) {
      instance = new Mast();
    }
    return instance;
  }

  public void runMast(double power) {
    mastMotor.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Pot", pot.pidGet());
  }

  public void runMastTriggers(double leftTrigger, double rightTrigger) {
    //fix pot later  && getPot() < 0.85,
    if (leftTrigger > mastDeadzone && leftTrigger > rightTrigger&& getPot() < 0.85) {
      runMast(-leftTrigger);
    } else if (rightTrigger > mastDeadzone && rightTrigger > leftTrigger&& getPot() > 0.02) {
      runMast(rightTrigger);
    } else {
      runMast(0);
    }
  }

  public double getPot() {
    return pot.pidGet();
  }
}
