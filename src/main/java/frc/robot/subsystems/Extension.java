package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commandGroups.CollectionExtensionCommandGroup;
import frc.robot.commandGroups.HighPoleExtensionCommandGroup;
import frc.robot.commandGroups.LowPoleExtensionCommandGroup;
import frc.robot.commandGroups.RetractedExtensionCommandGroup;
import frc.robot.utilities.BrainSTEMSubsystem;

public class Extension extends SubsystemBase implements BrainSTEMSubsystem {

  public static final class ExtensionConstants {
    private static final double k_gearRatioMultiplication = 7.0/12.0;

    private static final int k_frontExtensionMotorID = 14; 
    private static final int k_backExtensionMotorID = 31; 
    private static final int k_extensionServoID = 9;
    private static final double k_proportional = 0.00008;
    private static final double k_integral = 0;
    private static final double k_derivative = 0;
    private static final int k_retractedTelescopeValue = 0; //20000
    private static final int k_collectionTelescopeValue = (int)((220000/1.8) * k_gearRatioMultiplication);
    private static final int k_lowPoleTelescopeValue = (int) ((int)(162000 * k_gearRatioMultiplication) * 0.98); //*1.1 
    private static final int k_highPoleTelescopeValue = (int) ((int)((240000 * k_gearRatioMultiplication)) * 1.20);
    private static final int k_telescopeTolerance = (int)(1000 * k_gearRatioMultiplication);
    private static final double k_telescopeMaxPower = 1.00;
    public static final int k_backMotorOffRatchetValue = (int) ((75000 * k_gearRatioMultiplication) / 2); // FIXME
    public static final double k_backOffMotorSpeed = -0.01; // FIXME
  }

  public enum RatchetPosition {
    ENGAGED,
    DISENGAGED
  }

  public enum TelescopePosition {
    RETRACTED,
    COLLECTION,
    GROUND_COLLECTION,
    LOW_POLE,
    HIGH_POLE,
    TRANSITION
  }

  public boolean m_telescopeBackOff = false;

  public RatchetPosition m_ratchetState = RatchetPosition.ENGAGED;
  public TelescopePosition m_telescopeState = TelescopePosition.RETRACTED;

  TalonFX m_backMotor;
  TalonFX m_frontMotor;

  Servo m_ratchetServo;
  PIDController m_telescopePIDController;
  public int m_telescopeSetPoint = 0;
  public int m_unlockPosition = 0;
  private RetractedExtensionCommandGroup m_retractedExtensionCommandGroup;
  private CollectionExtensionCommandGroup m_collectionExtensionCommandGroup;
  private LowPoleExtensionCommandGroup m_lowPoleExtensionCommandGroup;
  private HighPoleExtensionCommandGroup m_highPoleExtensionCommandGroup;
  private boolean m_enableExtensionPeriodic = false;

  public Extension() {
    this.m_retractedExtensionCommandGroup = new RetractedExtensionCommandGroup(this);
    this.m_collectionExtensionCommandGroup = new CollectionExtensionCommandGroup(this);
    this.m_lowPoleExtensionCommandGroup = new LowPoleExtensionCommandGroup(this);
    this.m_highPoleExtensionCommandGroup = new HighPoleExtensionCommandGroup(this);

    m_backMotor = new TalonFX(ExtensionConstants.k_backExtensionMotorID);
    m_frontMotor = new TalonFX(ExtensionConstants.k_frontExtensionMotorID);

    m_ratchetServo = new Servo(ExtensionConstants.k_extensionServoID);
    m_telescopePIDController = new PIDController(ExtensionConstants.k_proportional, ExtensionConstants.k_integral,
        ExtensionConstants.k_derivative);
    m_telescopePIDController.setTolerance(ExtensionConstants.k_telescopeTolerance);
    // extensionServo.setBounds(1200, 125, 1100, 75, 1000);


  }

  @Override
  public void initialize() {
    m_backMotor.set(ControlMode.PercentOutput, 0);
    m_telescopeBackOff = false;
    m_telescopeSetPoint = 0;
    m_unlockPosition = 0;
    resetEncoder();
    m_telescopeState = TelescopePosition.RETRACTED;
    m_ratchetState = RatchetPosition.ENGAGED;
    enablePeriodic();
  }

  @Override
  public void enablePeriodic() {
    m_enableExtensionPeriodic = true;
  }

  @Override
  public void disablePeriodic() {
    m_enableExtensionPeriodic = false;
  }

  public void resetEncoder() {
    m_backMotor.setSelectedSensorPosition(0);
  }

  public void turnOffExtensionMotor() {
    m_backMotor.set(ControlMode.PercentOutput, 0);
  }

  public void ratchetDisengage() {
    m_ratchetServo.set(0.7);
  }

  public void ratchetEngage() {
    m_ratchetServo.set(0.6);
  }

  private void setRatchetState() {
    switch (m_ratchetState) {
      case ENGAGED:
        ratchetEngage();
        break;
      case DISENGAGED:
        ratchetDisengage();
        break;
    }
  }

  public void scheduleRetracted() {
    m_telescopeState = TelescopePosition.RETRACTED;
    scheduleTelescopeCommand();
  }

  public void scheduleCollection() {
    m_telescopeState = TelescopePosition.COLLECTION;
    scheduleTelescopeCommand();
  }

  public void scheduleLowPole() {
    m_telescopeState = TelescopePosition.LOW_POLE;
    scheduleTelescopeCommand();
  }

  public void scheduleHighPole() {
    m_telescopeState = TelescopePosition.HIGH_POLE;
    scheduleTelescopeCommand();
  }

  private void scheduleTelescopeCommand() {
    m_unlockPosition = ExtensionConstants.k_backMotorOffRatchetValue;
    switch (m_telescopeState) {
      case RETRACTED:
        m_retractedExtensionCommandGroup.schedule();
        break;
      case COLLECTION:
        m_collectionExtensionCommandGroup.schedule();
        break;
      case LOW_POLE:
        m_lowPoleExtensionCommandGroup.schedule();
        break;
      case HIGH_POLE:
        m_highPoleExtensionCommandGroup.schedule();
        break;
    }
  }

  public TelescopePosition getM_telescopeState() {
    if (inTolerance((int) m_backMotor.getSelectedSensorPosition(), ExtensionConstants.k_retractedTelescopeValue,
        ExtensionConstants.k_telescopeTolerance)) {
      return TelescopePosition.RETRACTED;
    } else if (inTolerance((int) m_backMotor.getSelectedSensorPosition(),
        ExtensionConstants.k_collectionTelescopeValue, ExtensionConstants.k_telescopeTolerance)) {
      return TelescopePosition.COLLECTION;
    } else if (inTolerance((int) m_backMotor.getSelectedSensorPosition(),
        ExtensionConstants.k_lowPoleTelescopeValue, ExtensionConstants.k_telescopeTolerance)) {
      return TelescopePosition.LOW_POLE;
    } else if (inTolerance((int) m_backMotor.getSelectedSensorPosition(),
        ExtensionConstants.k_highPoleTelescopeValue, ExtensionConstants.k_telescopeTolerance)) {
      return TelescopePosition.HIGH_POLE;
    } else {
      return TelescopePosition.TRANSITION;
    }
  }

  public double getTelescopeMotorPosition() {
    return m_backMotor.getSelectedSensorPosition();
  }

  private boolean inTolerance(int currentPosition, int targetPosition, int tolerance) {
    return (currentPosition > (targetPosition - tolerance)) &&
        (currentPosition < (targetPosition + tolerance));
  }

  private void setTelescopeState() {
    switch (m_telescopeState) {
      case RETRACTED:
        m_telescopeSetPoint = ExtensionConstants.k_retractedTelescopeValue;
        break;
      case COLLECTION:
        m_telescopeSetPoint = ExtensionConstants.k_collectionTelescopeValue;
        break;
      case GROUND_COLLECTION:
        m_telescopeSetPoint = (int) (ExtensionConstants.k_collectionTelescopeValue * 0.7);
        break;
      case LOW_POLE:
        m_telescopeSetPoint = ExtensionConstants.k_lowPoleTelescopeValue;
        break;
      case HIGH_POLE:
        m_telescopeSetPoint = ExtensionConstants.k_highPoleTelescopeValue;
        break;
    }
  }

  private void updateWithPID() {
    m_backMotor.setNeutralMode(NeutralMode.Brake);
    double k_MaxPower = 0;

    if (inTolerance((int) getTelescopeMotorPosition(), m_telescopeSetPoint, ExtensionConstants.k_telescopeTolerance)) {
      k_MaxPower = ExtensionConstants.k_telescopeMaxPower;
    } else {
      k_MaxPower = ExtensionConstants.k_telescopeMaxPower;
    }

    SmartDashboard.putNumber("E - k_MaxPower", k_MaxPower);

    if (m_telescopeState != TelescopePosition.RETRACTED) {
      m_backMotor.set(
        TalonFXControlMode.PercentOutput,
        MathUtil.clamp(
            m_telescopePIDController.calculate(m_backMotor.getSelectedSensorPosition(), m_telescopeSetPoint),
            -k_MaxPower, k_MaxPower));
    } else {
      m_backMotor.set(
        TalonFXControlMode.PercentOutput,
        (MathUtil.clamp(
            m_telescopePIDController.calculate(m_backMotor.getSelectedSensorPosition(), m_telescopeSetPoint),
            -k_MaxPower, k_MaxPower)));
    } 
    
        


  }

  public Command resetExtensionBase() {
    return runOnce(() -> {
      m_frontMotor.setSelectedSensorPosition(0.0);
      m_backMotor.setSelectedSensorPosition(0.0);
    });
  }

  @Override
  public void periodic() {
    if (m_enableExtensionPeriodic) {
      setRatchetState();
      setTelescopeState();
      updateWithPID();
      m_frontMotor.follow(m_backMotor);

      // Extensoin TELEMETRY ////////////////////////////////////////////////////////////////////////////////
      SmartDashboard.putNumber("E - Telescope Set Point", m_telescopeSetPoint);
      SmartDashboard.putNumber("E - Telescope Current Position", m_backMotor.getSelectedSensorPosition());
    }
  }

  @Override
  public void simulationPeriodic() {

  }
}
