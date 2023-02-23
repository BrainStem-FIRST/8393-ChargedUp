package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commandGroups.CollectionExtensionCommandGroup;
import frc.robot.commandGroups.HighPoleExtensionCommandGroup;
import frc.robot.commandGroups.LowPoleExtensionCommandGroup;
import frc.robot.commandGroups.RetractedExtensionCommandGroup;
import frc.robot.utilities.BrainSTEMSubsystem;


public class Extension extends SubsystemBase implements BrainSTEMSubsystem {

  private static final class ExtensionConstants {
    private static final int extensionMotorID = 14; 
    private static final int extensionServoID = 9; 
    private static final double kproportional = 0.1; //FIXME //rename with ks snake case
    private static final double INTEGRAL = 1; //FIXME
    private static final double DERIVATIVE = 0; //FIXME
    private static final int RETRACTED_TELESCOPE_VALUE = 0;
    private static final int COLLECTION_TELESCOPE_VALUE = 5000;
    private static final int LOW_POLE_TELESCOPE_VALUE = 10000;
    private static final int HIGH_POLE_TELESCOPE_VALUE = 15000;
    private static final int TELESCOPE_TOLERANCE = 200;
    private static final double TELESCOPE_MAX_POWER = 0.35;
  }

  public enum RatchetPosition {
    ENGAGED,
    DISENGAGED
  }

  public enum TelescopePosition {
    RETRACTED,
    COLLECTION,
    LOW_POLE,
    HIGH_POLE,
    TRANSITION
  }

  public RatchetPosition ratchetState = RatchetPosition.ENGAGED;
  public TelescopePosition telescopeState = TelescopePosition.RETRACTED;
  
  TalonFX telescopeMotor;
  Servo ratchetServo;
  PIDController telescopePIDController;
  private int telescopeSetPoint = 0;
  private RetractedExtensionCommandGroup retractedExtensionCommandGroup;
  private CollectionExtensionCommandGroup collectionExtensionCommandGroup;
  private LowPoleExtensionCommandGroup lowPoleExtensionCommandGroup;
  private HighPoleExtensionCommandGroup highPoleExtensionCommandGroup;

  public Extension() {
    this.retractedExtensionCommandGroup = new RetractedExtensionCommandGroup(this);
    this.collectionExtensionCommandGroup = new CollectionExtensionCommandGroup(this);
    this.lowPoleExtensionCommandGroup = new LowPoleExtensionCommandGroup(this);
    this.highPoleExtensionCommandGroup = new HighPoleExtensionCommandGroup(this);
    telescopeMotor = new TalonFX(ExtensionConstants.extensionMotorID);
    telescopeMotor.setInverted(true);
    ratchetServo = new Servo(ExtensionConstants.extensionServoID);
    telescopePIDController = new PIDController(ExtensionConstants.kproportional, ExtensionConstants.INTEGRAL, ExtensionConstants.DERIVATIVE);
    telescopePIDController.setTolerance(ExtensionConstants.TELESCOPE_TOLERANCE);
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

  private void setRatchetState() {
      switch (ratchetState) {
      case ENGAGED:
        ratchetEngage();
        break;
      case DISENGAGED:
        ratchetDisengage();
        break;
    }
  }

  public void scheduleRetracted() {
    telescopeState = TelescopePosition.RETRACTED;
    scheduleTelescopeCommand();
  }

  public void scheduleCollection() {
    telescopeState = TelescopePosition.COLLECTION;
    scheduleTelescopeCommand();
  }

  public void scheduleLowPole() {
    telescopeState = TelescopePosition.LOW_POLE;
    scheduleTelescopeCommand();
  }

  public void scheduleHighPole() {
    telescopeState = TelescopePosition.HIGH_POLE;
    scheduleTelescopeCommand();
  }

  private void scheduleTelescopeCommand() {
    switch (telescopeState) {
      case RETRACTED:
        retractedExtensionCommandGroup.schedule();
        break;
      case COLLECTION:
        collectionExtensionCommandGroup.schedule();
        break;
      case LOW_POLE:
        lowPoleExtensionCommandGroup.schedule();
        break;
      case HIGH_POLE:
        highPoleExtensionCommandGroup.schedule();
        break;
    }
  }

  public TelescopePosition getTelescopeState() {
    if (inTolerance((int) telescopeMotor.getSelectedSensorPosition(), ExtensionConstants.RETRACTED_TELESCOPE_VALUE, ExtensionConstants.TELESCOPE_TOLERANCE)) {
      return TelescopePosition.RETRACTED;
    } else if (inTolerance((int) telescopeMotor.getSelectedSensorPosition(), ExtensionConstants.COLLECTION_TELESCOPE_VALUE, ExtensionConstants.TELESCOPE_TOLERANCE)) {
      return TelescopePosition.COLLECTION;
    } else if (inTolerance((int) telescopeMotor.getSelectedSensorPosition(), ExtensionConstants.LOW_POLE_TELESCOPE_VALUE, ExtensionConstants.TELESCOPE_TOLERANCE)) {
      return TelescopePosition.LOW_POLE;
    } else if (inTolerance((int) telescopeMotor.getSelectedSensorPosition(), ExtensionConstants.HIGH_POLE_TELESCOPE_VALUE, ExtensionConstants.TELESCOPE_TOLERANCE)) {
      return TelescopePosition.HIGH_POLE;
    } else {
      return TelescopePosition.TRANSITION;
    }
      
  }

  public double getTelescopeMotorPosition() {
    return telescopeMotor.getSelectedSensorPosition();
  }

  private boolean inTolerance(int currentPosition, int targetPosition, int tolerance) {
    return (currentPosition > (targetPosition - tolerance)) &&
      (currentPosition < (targetPosition + tolerance));
  }

  private void setTelescopeState() {
      switch (telescopeState) {
        case RETRACTED:
        telescopeSetPoint = ExtensionConstants.RETRACTED_TELESCOPE_VALUE;
        break;
      case COLLECTION:
        telescopeSetPoint = ExtensionConstants.COLLECTION_TELESCOPE_VALUE;
        break;
      case LOW_POLE:
        telescopeSetPoint = ExtensionConstants.LOW_POLE_TELESCOPE_VALUE;
        break;
      case HIGH_POLE:
        telescopeSetPoint = ExtensionConstants.HIGH_POLE_TELESCOPE_VALUE;
        break;
    }
  }

  private void updateWithPID(){
    telescopeMotor.setNeutralMode(NeutralMode.Coast);
    telescopeMotor.set(
      TalonFXControlMode.PercentOutput, 
      MathUtil.clamp(
        telescopePIDController.calculate(telescopeMotor.getSelectedSensorPosition(), telescopeSetPoint),
        -ExtensionConstants.TELESCOPE_MAX_POWER, ExtensionConstants.TELESCOPE_MAX_POWER
      )
    );
  }

  @Override
  public void periodic() {
    setRatchetState();
    setTelescopeState();
    SmartDashboard.putNumber("Telescope Set Point", telescopeSetPoint);
    SmartDashboard.putNumber("Telescope Current Position", telescopeMotor.getSelectedSensorPosition());
    if(telescopeSetPoint > telescopeMotor.getSelectedSensorPosition()) {
      SmartDashboard.putBoolean("Telescope Motor Power", false);
      // telescopeMotor.set(ControlMode.PercentOutput, 0);
      // telescopeMotor.setNeutralMode(NeutralMode.Coast);
      // updateWithPID();
    } else {
      SmartDashboard.putBoolean("Telescope Motor Power", true);
      // updateWithPID();
    }
  }

  @Override
  public void simulationPeriodic() {

  }
}
