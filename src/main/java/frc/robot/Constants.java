/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;

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

    public static final double kS = 0.702;
    public static final double kV = 5.3;
    public static final double kA = 0.341;
    public static final double kP = 0.00175;
    public static final double kD = 0;
    public static final double rSquared = 1.0;

    public static final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(kS, kV, kA);

    public static final double maxVoltageAuto = 11;

    public static final int countPerRevolution = 4096;
    public static final double wheelDiameterMeters = 0.1524;
    public static final double wheelCircumferenceMeters = wheelDiameterMeters * Math.PI;

    // If Measured in meters trackWidth should be 0.58m
    public static final double trackWidth = 0.5792883856032899;
    public static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
        trackWidth);

    public static final DifferentialDriveVoltageConstraint VOLTAGE_CONSTRAINT =
        new DifferentialDriveVoltageConstraint(feedForward, kinematics, maxVoltageAuto);

    public static final double maxVelocityMetersPerSecond = 1;
    public static final double maxAccelerationMetersPerSecondSquared = 1;

    // These are standard m/s. Values provided by wpilib docs
    public static final double ramseteB = 2;
    public static final double ramseteZ = 0.7;

    public static final double turnP = 0.1;
    public static final double turnI = 0;
    public static final double turnD = 0;

    public static final double turnToleranceDeg = 0.5;
    public static final double turnRateToleranceDegPerS = 5;

    public static final double rotateDeadzone = 0.1;
    public static final double shiftPointMetersPerSecond = 1.5;
  }

  public static final class WinchConstants {
    public static final int winchMotor1 = 00;
    public static final int winchMotor2 = 00;
    public static final double winchDefaultMotorPower = 0.5;
  }

  public static final class IntakeConstants {

    public static final int jointMotorPort = 0;
    public static final int intakeMotorPort = 1;

    public static final double defaultIntakePower = 0.2;

    public static final int lowerPosition = -200;
    public static final int higherPosition = 0;

    public static final int limitSwitchPort = 01;

    public static final double kP = 0.03;
    public static final double kD = 0.00;
  }

  public static final class MagazineConstants {

    public static final int topMasterPort = 00;
    public static final int bottomMasterPort = 00;
    public static final int pistonHardStopForwardChannel = 00;
    public static final int pistonHardStopReverseChannel = 00;
    public static final double defaultMagazinePower = 0.2;
  }

  public static final class MastConstants {

    public static final int mastMotorPort = 00;
    public static final double mastDefaultMotorPower = 0.5;
  }

  public static final class OperatorInputConstants {

    public static final int driveControllerPort = 0;
    public static final int altControllerPort = 1;
  }

  public static final class PneumaticConstants {

    public static final int compressorPort = 0;

    public static final int[] shifterPorts = {0, 7};
  }

  public static final class SensorConstants {

    public static final int limitSwitchPort = 0;
  }

  public static final class ShooterConstants {

    public static final int leftShooterNeoPort = 2;
    public static final int rightShooterNeoPort = 3;
    public static final double flyWheelRadius = 2;
    public static final double shooterOpenLoopThreshold = 0.5;

    public static final double shooterClosedLoopThreshold = 0.95;
    public static final double shooterTightClosedLoopThreshold = 0.98;

    public static final double targetHeight = 5;
    public static final double shooterHeight = 4;

    //temporary values
    public static double shooterP = 0.0011;
    public static double shooterI = 0;
    public static double shooterD = 4;
    public static double shooterF = 0.00017;
  }

  public static final class WallOfFleshConstants {

    public static final int wallOfFleshMotorPort = 00;
    public static final double spinnerRadius = 50000;
    public static final double spinnerPower = 0.3;
    public static final double wallOfFleshSpinnerCircumference = 3;
    public static final double wallOfFleshCircumference = 100;
    public static final double goalSpinCount = 0.5;
    public static final int[] wallOfFleshSolenoidPorts = {0, 7};
  }
}