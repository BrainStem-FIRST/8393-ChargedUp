package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commandGroups.HighPoleExtensionCommandGroup;
import frc.robot.utilities.BrainSTEMSubsystem;


public class Extension extends SubsystemBase implements BrainSTEMSubsystem {

  private static final class ExtensionConstants {
    private static final int extensionMotorID = 14; 
    private static final int extensionServoID = 9; 
    private static final double kproportional = 0.1; //FIXME //rename with ks snake case
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
  private HighPoleExtensionCommandGroup highPoleExtensionCommandGroup;

  public Extension() {
    this.highPoleExtensionCommandGroup = new HighPoleExtensionCommandGroup(this);
    telescopeMotor = new TalonFX(ExtensionConstants.extensionMotorID);
    ratchetServo = new Servo(ExtensionConstants.extensionServoID);
    telescopePIDController = new PIDController(ExtensionConstants.kproportional, ExtensionConstants.INTEGRAL, ExtensionConstants.DERIVATIVE);
    // extensionServo.setBounds(1200, 125, 1100, 75, 1000);
  }

  @Override
  public void initialize(){
    resetEncoder();
  }

  public void resetEncoder(){
    telescopeMotor.setSelectedSensorPosition(0);
  }

  public void ratchetDisengage(){
    ratchetServo.set(0.0);
  }

  public void ratchetEngage(){
    ratchetServo.set(1.0);
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

  public void scheduleHighPole() {
    telescopeState = TelescopePosition.HIGH_POLE;
    scheduleTelescopeCommand();
  }

  private void scheduleTelescopeCommand() {
    switch (telescopeState) {
      case RETRACTED:
        telescopeSetPoint = 0; //make these constants //FIXME
        break;
      case COLLECTION:
        telescopeSetPoint = 100;
        break;
      case LOW_POLE:
        telescopeSetPoint = 200;
        break;
      case HIGH_POLE:
        highPoleExtensionCommandGroup.schedule();
        break;
    }

  }

  private void telescopeControl() { //set telescope state FIXME
      switch (telescopeState) {
      case RETRACTED:
        telescopeSetPoint = 0; //make these constants //FIXME
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
    telescopeMotor.setNeutralMode(NeutralMode.Brake);
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
      telescopeMotor.set(ControlMode.PercentOutput, 0);
      telescopeMotor.setNeutralMode(NeutralMode.Coast);
    } else {
      updateWithPID();
    }
  }

  @Override
  public void simulationPeriodic() {

  }
}
