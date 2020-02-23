/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.higherPosition;
import static frc.robot.Constants.IntakeConstants.intakeMotorPort;
import static frc.robot.Constants.IntakeConstants.jointMotorPort;
import static frc.robot.Constants.IntakeConstants.kD;
import static frc.robot.Constants.IntakeConstants.kP;
import static frc.robot.Constants.IntakeConstants.limitSwitchPort;
import static frc.robot.Constants.IntakeConstants.lowerPosition;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private static Intake instance;
  private static WPI_TalonSRX jointMotor;
  private static VictorSP intakeMotor;
  private PivotState pivotState;
  private DigitalInput limitSwitch;

  public enum PivotState {
    up,
    down
  }

  private Intake() {
    try {
      jointMotor = new WPI_TalonSRX(jointMotorPort);
    } catch (RuntimeException ex) {
      DriverStation
          .reportError("Error Instantiating Intake Motor Controllers: " + ex.getMessage(), true);
    }
    intakeMotor = new VictorSP(intakeMotorPort);

    try {
      limitSwitch = new DigitalInput(limitSwitchPort);
    } catch (RuntimeException ex) {
      DriverStation
          .reportError("Oh boy, limitswitch machine broke: " + ex.getMessage(), true);
    }

    TalonSRXConfiguration talonConfig = new TalonSRXConfiguration();
    talonConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;
    talonConfig.slot0.kP = kP;
    talonConfig.neutralDeadband = 0.0;
    talonConfig.slot0.kI = 0.0;
    talonConfig.slot0.kD = kD;
    talonConfig.slot0.integralZone = 400;
    talonConfig.slot0.closedLoopPeakOutput = 1.0;

    jointMotor.configAllSettings(talonConfig);

    zeroEncoder();

    pivotState = PivotState.up;
  }

  /**
   * Makes Intake a singleton.
   *
   * @return instance of intake
   */
  public static Intake getInstance() {
    if (instance == null) {
      instance = new Intake();
    }
    return instance;
  }

  public void lowerIntake() {
    jointMotor.set(ControlMode.Position, lowerPosition);
  }

  public void raiseIntake() {
    jointMotor.set(ControlMode.Position, higherPosition);
  }

  public void runIntake(double power) {
    intakeMotor.set(power);
  }

  public PivotState getPivotState() {
    return pivotState;
  }

  /**
   * Sets the state of the Pivot to current state.
   *
   * @param currentState the state of the Pivot.
   */
  public void setPivotState(PivotState currentState) {
    pivotState = currentState;
  }

  public boolean getLimitSwitch(){
    return limitSwitch.get();
  }

  public void zeroEncoder() {
    jointMotor.setSelectedSensorPosition(0, 0, 10);
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Joint State: ", pivotState.toString());
  }
}
