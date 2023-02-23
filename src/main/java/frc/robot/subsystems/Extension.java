package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.BrainSTEMSubsystem;


public class Extension extends SubsystemBase implements BrainSTEMSubsystem {

  private static final class ExtensionConstants {
    private static final int extensionMotorID = 14; 
    private static final int extensionServoID = 9; 
    private static final double PROPORTIONAL = 0.1; //FIXME
    private static final double INTEGRAL = 1; //FIXME
    private static final double DERIVATIVE = 0; //FIXME
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
  
  TalonFX telescopeMotor;
  Servo ratchetServo;
  PIDController telescopePIDController;
  private int telescopeSetPoint = 0;

  public Extension() {
    telescopeMotor = new TalonFX(ExtensionConstants.extensionMotorID);
    ratchetServo = new Servo(ExtensionConstants.extensionServoID);
    telescopePIDController = new PIDController(ExtensionConstants.PROPORTIONAL, ExtensionConstants.INTEGRAL, ExtensionConstants.DERIVATIVE);
    // extensionServo.setBounds(1200, 125, 1100, 75, 1000);
  }

  @Override
  public void initialize(){
    resetEncoder();
  }

  public void resetEncoder(){
    telescopeMotor.setSelectedSensorPosition(0);
  }

  public void fullExtend(){

  }

  public void ratchetDisengage(){
    ratchetServo.set(0.0);
  }

  public void ratchetEngage(){
    ratchetServo.set(1.0);
  }


  public void extend(double distance){

  }

  private void ratchetControl() {
      switch (ratchetState) {
      case ENGAGED:
        ratchetEngage();
        break;
      case DISENGAGED:
        ratchetDisengage();
        break;
    }
  }

  private void telescopeControl() {
      switch (telescopeState) {
      case RETRACTED:
        telescopeSetPoint = 0;
        break;
      case COLLECTION:
        telescopeSetPoint = 100;
        break;
      case LOW_POLE:
        telescopeSetPoint = 200;
        break;
      case HIGH_POLE:
        telescopeSetPoint = 300;
        break;
    }
  }

  private void updateWithPID(){
    telescopeMotor.set(
      TalonFXControlMode.PercentOutput, 
      telescopePIDController.calculate(telescopeMotor.getSelectedSensorPosition(), telescopeSetPoint)
    );
  }

  @Override
  public void periodic() {
    ratchetControl();
    telescopeControl();
    if(telescopeSetPoint > telescopeMotor.getSelectedSensorPosition()){
      SmartDashboard.putBoolean("Extension Coast", true);
      telescopeMotor.set(ControlMode.PercentOutput, 0);
      telescopeMotor.setNeutralMode(NeutralMode.Coast);
    } else {
      SmartDashboard.putBoolean("Extension Power", true);
      updateWithPID();
    }
  }

  @Override
  public void simulationPeriodic() {

  }
}
