package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.RobotContainer;


public class runIntakeToggle extends CommandBase {

  Intake intake;

  public runIntakeToggle(){
    intake = Intake.getInstance();
  }

  public void runIntake(boolean toggle){
    if(toggle){
      Intake.runIntake();
    }
    else{
      Intake.stopIntake();
    }
  }

  public void initialize(){

  }

  public void execute(){

  }

  public boolean isFinished(){
    return true;
  }

}




