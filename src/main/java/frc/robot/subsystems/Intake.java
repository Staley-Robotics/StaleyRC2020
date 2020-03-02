/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.defaultIntakePower;
import static frc.robot.Constants.IntakeConstants.defaultMotorJointPower;
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
  //private static WPI_TalonSRX jointMotor;
  private static VictorSP intakeMotor;
  private JointState jointState;
  private DigitalInput limitSwitch;

  public void runIntakeJoint(double motorPower) {

    boolean limitSwitchHit = limitSwitch.get();

  }

//  public void forceZeroEncoder() {
//    while(!limitSwitch.get()){
//      jointMotor.set(defaultMotorJointPower * 0.1);
//    }
//  }

  public enum JointState {
    up,
    down
  }

  private Intake() {
    try {
      //jointMotor = new WPI_TalonSRX(jointMotorPort);
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

    jointState = JointState.up;
  }

  public static Intake getInstance() {
    if (instance == null) {
      instance = new Intake();
    }
    return instance;
  }



  public void runIntake(double power) {
    intakeMotor.set(power);
  }

  public boolean getLimitSwitch() {
    return limitSwitch.get();
  }



  @Override
  public void periodic() {
    SmartDashboard.putString("Joint State: ", jointState.toString());
    SmartDashboard.putBoolean("Intake limit switch: ", limitSwitch.get());
  }

  public void checkLimitSwitch() {
  }
}
