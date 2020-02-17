package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;


/**
 * Code Copied from FRC225 RI3D.
 */
public class ColorMatcher {

  //Change the I2C port below to match the connection of your color sensor
  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

  private final ColorMatch colorMatcher = new ColorMatch();

  public static final Color kBlueTarget = ColorMatch.makeColor(0.136, 0.412, 0.450);
  public static final Color kGreenTarget = ColorMatch.makeColor(0.196, 0.557, 0.246);
  public static final Color kRedTarget = ColorMatch.makeColor(0.475, 0.371, 0.153);
  public static final Color kYellowTarget = ColorMatch.makeColor(0.293, 0.561, 0.144);

  private ColorMatchResult matchedResult = new ColorMatchResult(Color.kBlack, 0);

  // Rev Color threshold
  // blue 0.143, 0.427, 0.429
  // green 0.197, 0.561, 0.240
  // red 0.561, 0.232, 0.114
  // yellow 0.361, 0.524, 0.113

  /**
   * Constructor for ColorMatcher.
   */
  public ColorMatcher() {
    colorMatcher.addColorMatch(kBlueTarget);
    colorMatcher.addColorMatch(kGreenTarget);
    colorMatcher.addColorMatch(kRedTarget);
    colorMatcher.addColorMatch(kYellowTarget);

    colorMatcher.setConfidenceThreshold(0.80);
  }

  /**
   * The method GetColor() returns a normalized color value from the sensor and can be useful if
   * outputting the color to an RGB LED or similar. To read the raw color, use GetRawColor(). The
   * color sensor works best when within a few inches from an object in well lit conditions (the
   * built in LED is a big help here!). The farther an object is the more light from the
   * surroundings will bleed into the measurements and make it difficult to accurately determine its
   * color.
   */
  public Color get_color() {

    Color detectedColor = colorSensor.getColor();

    String colorString;
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);
    return match.color;
  }
}