/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.higherPosition;
import static frc.robot.Constants.IntakeConstants.intakeMotorPort;
import static frc.robot.Constants.IntakeConstants.jointDeadzone;
import static frc.robot.Constants.IntakeConstants.jointMotorPort;
import static frc.robot.Constants.IntakeConstants.kD;
import static frc.robot.Constants.IntakeConstants.kP;
import static frc.robot.Constants.IntakeConstants.limitSwitchPort;
import static frc.robot.Constants.IntakeConstants.lowerPosition;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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
  private JointState jointState;
  private DigitalInput limitSwitch;

  public void runIntakeJoint(double motorPower) {
    boolean limitSwitchHit = limitSwitch.get();

    if (motorPower < -jointDeadzone && !limitSwitchHit) {
      jointMotor.set(motorPower);
    } else if (motorPower > jointDeadzone) {
      jointMotor.set(motorPower);
    } else {
      jointMotor.set(0);
    }
  }

  public enum JointState {
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
      System.out.println("Initialized limit switch");
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
    talonConfig.slot0.closedLoopPeakOutput = 0.5;

    jointMotor.configAllSettings(talonConfig);
    jointMotor.setNeutralMode(NeutralMode.Brake);
    jointMotor.setInverted(true);
    zeroEncoder();

    jointState = JointState.up;
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
    jointState = JointState.down;
    jointMotor.set(ControlMode.Position, lowerPosition);
    jointState = JointState.down;
  }

  public void raiseIntake() {
    jointState = JointState.up;
    jointMotor.set(ControlMode.Position, higherPosition);
    jointState = JointState.up;
  }

  /**
   * Toggles pivot arm's PID set point between high/low.
   */
  public void toggleIntake() {
    if (jointState == JointState.up) {
      lowerIntake();
    } else if (jointState == JointState.down) {
      raiseIntake();
    }
  }

  public void runIntake(double power) {
    intakeMotor.set(power);
  }

  public JointState getJointState() {
    return jointState;
  }

  /**
   * Sets the state of the Pivot to current state.
   *
   * @param currentState the state of the Pivot.
   */
  public void setJointState(JointState currentState) {
    jointState = currentState;
  }

  public boolean getLimitSwitch() {
    return limitSwitch.get();
  }

  public void zeroEncoder() {
    jointMotor.setSelectedSensorPosition(0, 0, 10);
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Joint State: ", jointState.toString());
    SmartDashboard.putBoolean("Intake limit switch: ", limitSwitch.get());
  }
}
