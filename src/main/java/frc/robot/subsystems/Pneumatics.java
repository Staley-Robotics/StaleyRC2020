
package frc.robot.subsystems;

import static frc.robot.Constants.PneumaticConstants.compressorPort;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {

  private static Pneumatics instance;
  private Compressor compressor;

  private Pneumatics() {
    compressor = new Compressor(compressorPort);
  }

  public static Pneumatics getInstance() {
    if (instance == null) {
      instance = new Pneumatics();
    }
    return instance;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Compressor", compressor.getClosedLoopControl());
  }

  public void runCompressor() {
    compressor.setClosedLoopControl(true);
  }

  public void stopCompressor() {
    compressor.setClosedLoopControl(false);
  }

  public void compressorToggle() {
    if (compressor.getClosedLoopControl()) {
      stopCompressor();
    } else {
      runCompressor();
    }
  }
}