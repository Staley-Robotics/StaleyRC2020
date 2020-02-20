/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WallOfFleshConstants;

/**
 * Wall of Flesh subsystem. 1 775 powered by a talon srx with an encoder a piston color sensor.
 */
public class WallOfFlesh extends SubsystemBase {

  private static Color goalColor;
  private static ColorMatcher colorMatcher;
  private static Color[] Colors;
  private static WallOfFlesh instance;

  private static WPI_TalonSRX WofMotor;
  private DoubleSolenoid wofPiston;

  /**
   * Constructor.
   */
  public WallOfFlesh() {
    colorMatcher = new ColorMatcher();
    colorMatcher.get_color();
    Colors = new Color[]{ColorMatcher.kGreenTarget,
        ColorMatcher.kRedTarget,
        ColorMatcher.kYellowTarget,
        ColorMatcher.kBlueTarget};
    WofMotor = new WPI_TalonSRX(WallOfFleshConstants.wallOfFleshMotorPort);
    wofPiston = new DoubleSolenoid(0, 7);
  }

  /**
   * Makes WOF a singleton.
   *
   * @return WOF instance.
   */
  public static WallOfFlesh getInstance() {
    if (instance == null) {
      instance = new WallOfFlesh();
    }
    return instance;
  }

  public Color[] getColors() {

    return Colors;
  }

  public void setGoalColor(Color goalColor) {
    this.goalColor = goalColor;
  }

  public void runWOFSpinner(double power) {

    WofMotor.set(power);
  }

  public Color getCurrentColor() {
    return colorMatcher.get_color();
  }

  public void raiseWof() {
    wofPiston.set(Value.kForward);
  }

  public void lowerWof() {
    wofPiston.set(Value.kReverse);
  }

  /**
   * Spins the WOF spinner distance in meters.
   *
   * @param distance distance in meters
   */
  public void spinDistance(double distance) {
    int goalEncoderTick = (int) ((distance / (2 * Math.PI * WallOfFleshConstants.spinnerRadius))
        * 4096);
    WofMotor.getSelectedSensorPosition();
    WofMotor.set(ControlMode.Position, WofMotor.getSelectedSensorPosition() + goalEncoderTick);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
