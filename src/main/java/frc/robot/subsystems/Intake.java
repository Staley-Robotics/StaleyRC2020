/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.intakeMotorPort;
import static frc.robot.Constants.IntakeConstants.jointMotorPort;
import static frc.robot.Constants.SensorConstants.limitSwitchPort;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private static Intake instance;
  private static VictorSP jointMotor;
  private static VictorSP intakeMotor;
  private DigitalInput limitSwitch;
  private PivotState pivotState;

  public enum PivotState {
    up,
    down
  }

  public Intake() {
    limitSwitch = new DigitalInput(limitSwitchPort);
    jointMotor = new VictorSP(jointMotorPort);
    intakeMotor = new VictorSP(intakeMotorPort);

    pivotState = PivotState.up;
  }

  public static Intake getInstance() {
    if (instance == null) {
      instance = new Intake();
    }
    return instance;
  }

  public void runJoint(double power) {
    jointMotor.set(power);
  }

  public void runIntake(double power) {
    intakeMotor.set(power);
  }

  public boolean getLimitSwitch() {
    return limitSwitch.get();
  }

  public PivotState getPivotState() {
    return pivotState;
  }

  /**
   * Sets the state of the Pivot opposite to what it currently is.
   * @param currentState the state of the Pivot.
   */
  public void setPivotState(PivotState currentState) {
    if (currentState == PivotState.down) {
      pivotState = PivotState.up;
    } else {
      pivotState = PivotState.down;
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Joint State: ", pivotState.toString());
  }
}
