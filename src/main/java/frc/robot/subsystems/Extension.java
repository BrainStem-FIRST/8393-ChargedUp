package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Extension extends SubsystemBase {

  private static final class ExtensionConstants {
    private static final int extensionMotorID = 69; //FIXME
    private static final int extensionServoID = 70; //FIXME
  }
  
  TalonFX extensionMotor;
  Servo extensionServo;

  public Extension() {
    extensionMotor = new TalonFX(ExtensionConstants.extensionMotorID);
    extensionServo = new Servo(ExtensionConstants.extensionServoID);
  }

  public void fullExtend(){

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
