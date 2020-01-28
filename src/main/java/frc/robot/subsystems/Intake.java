/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SensorConstants;

public class Intake extends SubsystemBase {

  private static Intake instance;
  private static VictorSP jointMotor;
  private static VictorSP intakeMotor;
  private DigitalInput limitSwitch;
  private PivotState pivotState;
  private int num;

  public enum PivotState {
    up,
    down
  }

  public Intake() {
    limitSwitch = new DigitalInput(SensorConstants.limitSwitchPort);
    jointMotor = new VictorSP(IntakeConstants.jointMotorPort);
    intakeMotor = new VictorSP(IntakeConstants.intakeMotorPort);

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

  public void setPivotStateDown() {
    pivotState = PivotState.down;
  }

  public void setPivotStateUp() {
    pivotState = PivotState.up;
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Joint State: ", pivotState.toString());
  }
}
