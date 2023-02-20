package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.BrainSTEMSubsystem;


public class Extension extends SubsystemBase implements BrainSTEMSubsystem {

  private static final class ExtensionConstants {
    private static final int extensionMotorID = 14; 
    private static final int extensionServoID = 9; 
  }
  
  TalonFX extensionMotor;
  Servo extensionServo;

  public Extension() {
    extensionMotor = new TalonFX(ExtensionConstants.extensionMotorID);
    extensionServo = new Servo(ExtensionConstants.extensionServoID);
    extensionServo.setBounds(2750, 125, 1375, 75, 0);
  }

  @Override
  public void initialize(){

  }

  public void fullExtend(){

  }

  public void extensionMotorOn(){
    extensionMotor.set(ControlMode.PercentOutput, 0.02);
  }

  public void moveServoToMin(){
    extensionServo.set(0);
  }

  public void moveServoToMax(){
    extensionServo.set(1);
  }


  public void extend(double distance){

  }

  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {

  }
}
