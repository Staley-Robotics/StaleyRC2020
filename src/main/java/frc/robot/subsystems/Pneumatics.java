
package frc.robot.subsystems;

import static frc.robot.Constants.PneumaticConstants.compressorPort;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Runs compressor.
 */
public class Pneumatics extends SubsystemBase {

  private static Pneumatics instance;
  private Compressor compressor;
  private CompressorState compressorState;

  public enum CompressorState {
    on,
    off
  }

  private Pneumatics() {
    compressor = new Compressor(compressorPort);

    compressorState = CompressorState.off;
  }


  /**
   * Makes Pneumatics a singleton.
   */
  public static Pneumatics getInstance() {
    if (instance == null) {
      instance = new Pneumatics();
    }
    return instance;
  }

  public void runCompressor() {
    compressor.setClosedLoopControl(true);
  }

  public void stopCompressor() {
    compressor.setClosedLoopControl(false);
  }

  public void compressorToggle() {
    if (compressorState == CompressorState.off) {
      runCompressor();
      compressorState = CompressorState.on;
    } else if (compressorState == CompressorState.on) {
      stopCompressor();
      compressorState = CompressorState.off;
    }
  }
}