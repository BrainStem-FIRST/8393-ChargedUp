package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.BrainSTEMSubsystem;


public class Extension extends SubsystemBase implements BrainSTEMSubsystem {

  private static final class ExtensionConstants {
    private static final int extensionMotorID = 14; 
    private static final int extensionServoID = 9; 
  }

  public enum RatchetPosition {
    ENGAGED,
    DISENGAGED
  }

  public enum TelescopePosition {
    RETRACTED,
    COLLECTION,
    LOW_POLE,
    HIGH_POLE
  }

  public RatchetPosition ratchetState = RatchetPosition.ENGAGED;
  public TelescopePosition telescopeState = TelescopePosition.RETRACTED;
  
  TalonFX extensionMotor;
  Servo extensionServo;

  public Extension() {
    extensionMotor = new TalonFX(ExtensionConstants.extensionMotorID);
    extensionServo = new Servo(ExtensionConstants.extensionServoID);
    // extensionServo.setBounds(1200, 125, 1100, 75, 1000);
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
    SmartDashboard.putNumber("Ratchet Servo Position", 0.5);
    extensionServo.setAngle(0.99);
  }

  public void moveServoToMax(){
    SmartDashboard.putNumber("Ratchet Servo Position", 0.51);
    extensionServo.set(0.98);
  }


  public void extend(double distance){

  }

  @Override
  public void periodic() {
    switch (ratchetState) {
      case ENGAGED:
        moveServoToMin();
        break;
      case DISENGAGED:
        moveServoToMax();
        break;
    }
  }

  @Override
  public void simulationPeriodic() {

  }
}
