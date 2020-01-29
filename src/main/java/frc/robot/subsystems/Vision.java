package frc.robot.subsystems;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Handles everything vision related on the robot. #78.5 tall on wall.
 */
public class Vision extends SubsystemBase {

  private static Vision instance;

  private NetworkTableInstance inst;
  private NetworkTable table;
  private NetworkTableEntry tapeProcessing;
  private NetworkTableEntry tapeYaw;
  private NetworkTableEntry videoTimeStamp;
  private NetworkTableEntry tapeDetected;
  private NetworkTableEntry distance;
  private NetworkTableEntry cameraPublisher;
  private NetworkTableEntry camera;
  private int count = 0;

  private Vision() {
    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("Vision Info");
    initializeNetworkTables();
  }
  /*
  /CameraPublisher
  /Camera
    streams=["mjpeg:http://roborio-0000-frc.local:1181/?action=stream"]
   */

  private void startCameraStream() {
    final String piAddress = "10.49.59.103";
    final int port = 1181;
    table.getEntry("/CameraPublisher/PiCamera/streams").setStringArray(
        new String[]{"mjpeg:http://" + piAddress + ":" + port + "/?action=stream"});
  }


  private void initializeNetworkTables() {
    setTapeProcessing(true);
    zeroYaw();
    zeroTimeStamp();
    setDistance(0.0);
    setTapeDetected(false);
    startCameraStream();
  }

  private void setTapeDetected(boolean detected) {
    tapeDetected = table.getEntry("tapeDetected");
    tapeDetected.setBoolean(false);
  }

  private void zeroYaw() {
    tapeYaw = table.getEntry("tapeYaw");
    tapeYaw.setNumber(0.0);
  }

  private void zeroTimeStamp() {
    videoTimeStamp = table.getEntry("videoTimeStamp");
    videoTimeStamp.setNumber(0.0);
  }

  /**
   * If tapeProcessing is true, vision track. If it is false, the camera stream will not be
   * affected.
   *
   * @param processing detect tape or not
   */
  private void setTapeProcessing(boolean processing) {
    tapeProcessing = table.getEntry("tapeProcessing");
    tapeProcessing.setBoolean(processing);
  }

  /**
   * Ensures vision is a singleton.
   *
   * @return
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
    getYaw(count);
    count++;
  }

  /**
   * you're mom.
   * @return
   */
  public double getYaw(int counter) {
    tapeYaw = table.getEntry("tapeYaw");
    //return tapeYaw.getDouble(0.0);
    SmartDashboard.putNumber("tapeYaw", counter);
    return tapeYaw.getDouble(1.0);
  }

  public double getDistance() {
    distance = table.getEntry("tapeDistance");
    return distance.getDouble(0.0);
  }

  public void setDistance(double num) {
    distance = table.getEntry("tapeDistance");
    distance.setNumber(num);
  }


  public boolean tapeDetected() {
    tapeDetected = table.getEntry("tapeDetected");
    return tapeDetected.getBoolean(false);
  }
}
