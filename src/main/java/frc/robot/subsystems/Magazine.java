/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.MagazineConstants.bottomMasterPort;
import static frc.robot.Constants.MagazineConstants.pistonHardStopForwardChannel;
import static frc.robot.Constants.MagazineConstants.pistonHardStopReverseChannel;
import static frc.robot.Constants.MagazineConstants.topMasterPort;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Magazine that holds 5 balls. It will run constantly while we are intaking. Once we get a ball, it
 * will be taken towards the shooter until it finds it can't move any more (either by collision with
 * our piston hard stop or another ball). Once we have 5 balls, our driver will stop the magazine
 * from running. When we shoot, we retract the piston hard stop and then shoot.
 */
public class Magazine extends SubsystemBase {

  private DoubleSolenoid pistonHardStop;
  private static Magazine instance;
  private VictorSP topMaster;
  private VictorSP bottomMaster;
  private PistonHardStopState pistonHardStopState;

  private enum PistonHardStopState {
    extended,
    retracted
  }

  /**
   * Constructor.
   */
  private Magazine() {
    try {
      topMaster = new VictorSP(topMasterPort);
      bottomMaster = new VictorSP(bottomMasterPort);
    } catch (RuntimeException ex) {
      DriverStation
          .reportError("Error Instantiating Magazine Motor Controllers: " + ex.getMessage(), true);
    }
    topMaster.setInverted(false);
    bottomMaster.setInverted(true);
    pistonHardStop = new DoubleSolenoid(pistonHardStopForwardChannel, pistonHardStopReverseChannel);
    extendHardStop();
  }

  /**
   * Makes Magazine a singleton.
   *
   * @return Magazine instance
   */
  public static Magazine getInstance() {
    if (instance == null) {
      instance = new Magazine();
    }
    return instance;
  }

  public void extendHardStop() {
    pistonHardStop.set(Value.kForward);
    pistonHardStopState = PistonHardStopState.extended;
  }

  public void retractHardStop() {
    pistonHardStop.set(Value.kReverse);
    pistonHardStopState = PistonHardStopState.retracted;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runMagazine(double speed) {
    topMaster.set(speed);
    bottomMaster.set(speed);
  }
}
