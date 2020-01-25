/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

  private static Intake instance;
  private static WPI_TalonSRX jointMotor;
  private static WPI_TalonSRX intakeMotor;

  private static double jointMotorPower;
  private static double intakeMotorPower;

  public Intake() {
    jointMotor = new WPI_TalonSRX(IntakeConstants.jointMotorPort);
    intakeMotor = new WPI_TalonSRX(IntakeConstants.intakeMotorPort);

    jointMotorPower = 1.0;
    intakeMotorPower = 1.0;
  }

  public static Intake getInstance() {
    if (instance == null) {
      instance = new Intake();
    }
    return instance;
  }

  public static void lowerJoint() {
    jointMotor.set(jointMotorPower);
  }

  public static void raiseJoint() {
    jointMotor.set(-jointMotorPower);
  }

  public static void stopJoint() {
    jointMotor.set(0);
  }

  public static void runIntake() {
    intakeMotor.set(intakeMotorPower);
  }

  public static void stopIntake() {
    intakeMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
