package frc.robot.enums;

public class XboxController {

  /**
   * Represents a digital dpad button on an XboxController.
   */
  public enum DpadButton {
    kDpadUp(0),
    kDpadDown(180),
    kDpadLeft(270),
    kDpadRight(90);

    @SuppressWarnings({"MemberName", "PMD.SingularField"})
    public final int value;

    DpadButton(int value) {
      this.value = value;
    }

    public int getValue() {
      return value;
    }
  }

}
