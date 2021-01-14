package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.intakeMotorPort;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private static Intake instance;
  private static VictorSP intakeMotor;


  private Intake() {
    intakeMotor = new VictorSP(intakeMotorPort);
  }

  public static Intake getInstance() {
    if (instance == null) {
      instance = new Intake();
    }
    return instance;
  }


  public void runIntake(double power) {
    intakeMotor.set(power);
  }

}