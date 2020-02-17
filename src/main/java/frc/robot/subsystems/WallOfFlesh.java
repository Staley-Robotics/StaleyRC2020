/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
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

  private static WPI_TalonSRX WOFMotor;

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
    WOFMotor = new WPI_TalonSRX(WallOfFleshConstants.wallOfFleshMotorPort);
  }

  public Color[] getColors() {
    return Colors;
  }

  public void setGoalColor(Color goalColor) {
    this.goalColor = goalColor;
  }

  public void runWOFSpinner(double power) {
    WOFMotor.set(power);
  }

  public Color getCurrentColor() {
    return colorMatcher.get_color();
  }

  /**
   * Spins the WOF spinner distance in meters.
   *
   * @param distance distance in meters
   */
  public void spinDistance(double distance) {
    int goalEncoderTick = (int) ((distance / (2 * Math.PI * WallOfFleshConstants.spinnerRadius))
        * 4096);
    WOFMotor.getSelectedSensorPosition();
    WOFMotor.set(ControlMode.Position, WOFMotor.getSelectedSensorPosition() + goalEncoderTick);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
