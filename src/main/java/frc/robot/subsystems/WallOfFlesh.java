/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;

public class WallOfFlesh extends SubsystemBase {

  /**
   * Creates a new Wall of Flesh. 1 775 powered by a talon srx with an encoder a piston color sensor
   * known wheel circumference
   */
  private static Color goalColor;
  private static ColorMatcher colorMatcher;
  private static Color[] Colors;

  private static WPI_TalonSRX WOFMotor;
  private static double distanceSpun;

  public WallOfFlesh() {
    colorMatcher = new ColorMatcher();
    colorMatcher.get_color();
    Colors = new Color[]{ColorMatcher.kGreenTarget,
        ColorMatcher.kRedTarget,
        ColorMatcher.kYellowTarget,
        ColorMatcher.kBlueTarget};
    WOFMotor = new WPI_TalonSRX(Constants.WOFConstants.WOFMotorPort);
    distanceSpun = 0;

  }

  public void setGoalColor(Color goalColor) {
    this.goalColor = goalColor;
  }

  public Color getCurrentColor() {
    return colorMatcher.get_color();
  }

  public void spinDistance(double distance) {

    int goalEncoderTick = (int) (
        Math.floor(distance / (2 * Math.PI * Constants.WOFConstants.spinnerRadius)) * 4096);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
