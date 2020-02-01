/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed to reduce verbosity.
 */
public final class Constants {

  public static final class DriveConstants {

    public static final int rMotorMasterPort = 1;
    public static final int rMotorFollower1Port = 9;
    public static final int rMotorFollower2Port = 10;

    public static final int lMotorMasterPort = 4;
    public static final int lMotorFollower1Port = 6;
    public static final int lMotorFollower2Port = 7;

    public static final double kS = 1.04;
    public static final double kV = 2.11;
    public static final double kA = 0.443;
    public static final double kP = 0.00217;
    public static final double kD = 0;
    public static final double rSquared = 0.996;

    // If Measured in meters right should be 0.58m
    public static final double trackWidth = 0.58;
    public static final DifferentialDriveKinematics kKinematics = new DifferentialDriveKinematics(
        trackWidth);

    // Will change.
    public static final double kMaxSpeedInchesPerSecond = 8;
    public static final double kMaxAccelerationInchesPerSecondSquared = 3;

    // These are standard m/s. Values provided by wpilib docs
    public static final double kRamseteB = 2;
    public static final double kRamseteZ = 0.7;
  }

  public static final class OperatorInputConstants {

    public static final int driveControllerPort = 0;
    public static final int altControllerPort = 1;
  }

  public static final class IntakeConstants {

    public static final int jointMotorPort = 0;
    public static final int intakeMotorPort = 1;

    public static final double defaultIntakePower = 0.2;
    public static final double defualtJointPower = 0.2;
  }

  public static final class SensorConstants {

    public static final int limitSwitchPort = 0;
  }
}