/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.FeederConstants.bottomMasterPort;
import static frc.robot.Constants.FeederConstants.topMasterPort;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Magazine extends SubsystemBase {
  private static Magazine instance;
  private WPI_TalonSRX topMaster;
  private WPI_TalonSRX bottomMaster;

  /**
   * Constructor.
   */
  public Magazine() {
    topMaster = new WPI_TalonSRX(topMasterPort);
    bottomMaster = new WPI_TalonSRX(bottomMasterPort);
    topMaster.setInverted(false);
    bottomMaster.setInverted(true);
  }

  /**
   * Makes Magazine a singleton.
   * @return Magazine instance
   */
  public static Magazine getInstance() {
    if (instance == null) {
      instance = new Magazine();
    }
    return instance;
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
