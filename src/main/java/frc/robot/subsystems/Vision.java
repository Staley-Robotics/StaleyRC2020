package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Handles everything vision related on the robot including networktable communications.
 */
public class Vision extends SubsystemBase {

  private static Vision instance;

  private final NetworkTable table;

  private final double bottomOfTargetHeight = 78.5;
  private final double fixedCameraHeight = 8.875;
  //Angle of the camera above horizontal. Must be accurately measured for distance calculation.
  private final double fixedCameraAngle = 17;

  private Vision() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    table = inst.getTable("chameleon-vision/Microsoft LifeCam HD-3000");
  }

  /**
   * Makes Vision a singleton.
   * @return Vision
   */
  public static Vision getInstance() {
    if (instance == null) {
      instance = new Vision();
    }
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Tape Detected", tapeDetected());
    if (tapeDetected()) {
      SmartDashboard.putNumber("Distance", calculateDistance(getPitch()));
    }

  }

  /**
   * If trackTape is true, vision track. If trackTape is false, the camera stream will not be
   * processed.
   *
   * @param trackTape track tape or not
   */
  public void setTapeProcessing(final boolean trackTape) {
    NetworkTableEntry targetProcessing = table.getEntry("driverMode");
    targetProcessing.setBoolean(trackTape);
  }

  /**
   * Check if processing sees a target.
   *
   * @return Target detected or not
   */
  public boolean tapeDetected() {
    NetworkTableEntry targetDetected = table.getEntry("isValid");
    return targetDetected.getBoolean(false);

  }

  /**
   * Calculates distance using trig and a pitch measurement. See https://docs.limelightvision.io/en/latest/cs_estimating_distance.html#using-a-fixed-angle-camera
   *
   * @return
   */
  public double calculateDistance(double pitch) {
    return (bottomOfTargetHeight - fixedCameraHeight) / (Math
        .tan(Math.toRadians(pitch + fixedCameraAngle)));
  }

  /**
   * Gets and returns pitch from chameleon vision network table.
   *
   * @return Calculated pitch
   */
  public double getPitch() {
    NetworkTableEntry targetPitch = table.getEntry("targetPitch");
    return targetPitch.getDouble(0.0);
  }

  /**
   * Gets and returns yaw from chameleon vision network table.
   *
   * @return Calculated yaw
   */
  public double getYaw() {
    NetworkTableEntry targetYaw = table.getEntry("targetYaw");
    return targetYaw.getDouble(0.0);
  }
}
