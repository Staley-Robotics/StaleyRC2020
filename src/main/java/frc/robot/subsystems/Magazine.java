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

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private Magazine() {
    topMaster = new VictorSP(topMasterPort);
    bottomMaster = new VictorSP(bottomMasterPort);
    topMaster.setInverted(false);
    bottomMaster.setInverted(true);
    pistonHardStop = new DoubleSolenoid(pistonHardStopForwardChannel, pistonHardStopReverseChannel);
    extendHardStop();
  }

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

  public void toggleHardStop() {
    if (pistonHardStopState == PistonHardStopState.retracted) {
      extendHardStop();
    } else if (pistonHardStopState == PistonHardStopState.extended) {
      retractHardStop();
    } else {
      throw new IllegalStateException("heck, toggleHardStop broke");
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("Mag Piston", pistonHardStopState.toString());
  }

  public void runMagazine(double speed) {
    topMaster.set(speed);
    bottomMaster.set(speed);
  }

}
