/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.MagazineConstants.defaultMagazinePower;
import static frc.robot.Constants.MagazineConstants.magLimitSwitchPort;
import static frc.robot.Constants.MagazineConstants.pistonHardStopForwardChannel;
import static frc.robot.Constants.MagazineConstants.pistonHardStopReverseChannel;
import static frc.robot.Constants.MagazineConstants.topMasterPort;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.magazine.RunMagazine;

/**
 * Magazine that holds 5 balls. It will run constantly while we are intaking. Once we get a ball, it
 * will be taken towards the shooter until it finds it can't move any more (either by collision with
 * our piston hard stop or another ball). Once we have 5 balls, our driver will stop the magazine
 * from running. When we shoot, we retract the piston hard stop and then shoot.
 */
public class Magazine extends SubsystemBase {

  private static Magazine instance;

  private DoubleSolenoid pistonHardStop;
  private VictorSP topMotor;
  private PistonHardStopState pistonHardStopState;
  private DigitalInput limitSwitch;
  private boolean oldLimitSwitch;
  private int ballCount;
  private boolean magazineEnabled;

  private Magazine() {
    topMotor = new VictorSP(topMasterPort);
    topMotor.setInverted(false);
    pistonHardStop = new DoubleSolenoid(pistonHardStopForwardChannel, pistonHardStopReverseChannel);
    limitSwitch = new DigitalInput(magLimitSwitchPort);
    ballCount = 0;
    magazineEnabled = true;
    oldLimitSwitch = false;
    extendHardStop();
  }

  private enum PistonHardStopState {
    extended,
    retracted
  }

  public static Magazine getInstance() {
    if (instance == null) {
      instance = new Magazine();
    }
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("Mag Piston", pistonHardStopState.toString());
    boolean limitSwitchPressed = limitSwitch.get();

//    if (limitSwitchPressed && ballCount == 3) {
//      magazineEnabled = false;
//    } else if (limitSwitchPressed && !oldLimitSwitch) {
//      magazineEnabled = true;
//      ballCount++;
//      oldLimitSwitch = limitSwitchPressed;
//    } else if (!limitSwitchPressed && oldLimitSwitch) {
//      magazineEnabled = false;
//      oldLimitSwitch = limitSwitchPressed;
//    } else if (!limitSwitchPressed) {
//      oldLimitSwitch = limitSwitchPressed;
//    }
//!limitswitchpressed && oldLimitSwitch, run magazine for 0.4
    if (!limitSwitchPressed && oldLimitSwitch) {
      new RunMagazine(defaultMagazinePower).withTimeout(0.6).andThen(new RunMagazine(0))
          .schedule();
    }
    oldLimitSwitch = limitSwitchPressed;
  }

  public void extendHardStop() {
    //pistonHardStop.set(Value.kForward);
    pistonHardStopState = PistonHardStopState.extended;
  }

  public void retractHardStop() {
    //pistonHardStop.set(Value.kReverse);
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

  public void resetBallCount() {
    ballCount = 0;
  }

  public int getBallCount() {
    return ballCount;
  }

  public void runMagazine(double speed) {
    if (magazineEnabled) {
      topMotor.set(speed);
    }
  }

  public void runMagazineForced(double speed) {
    resetBallCount();
    magazineEnabled = true;
    topMotor.set(speed);

  }

}
