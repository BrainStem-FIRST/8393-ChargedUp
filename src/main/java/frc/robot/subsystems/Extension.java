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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Extension extends SubsystemBase implements BrainSTEMSubsystem {

  public static final class ExtensionConstants {
    private static final int k_extensionMotorID = 14;
    private static final int k_extensionServoID = 9;
    private static final double k_proportional = 0.000125;
    private static final double k_integral = 0;
    private static final double k_derivative = 0;
    private static final int k_retractedTelescopeValue = 0;
    private static final int k_collectionTelescopeValue = 60000;
    private static final int k_lowPoleTelescopeValue = 150000;
    private static final int k_highPoleTelescopeValue = 245000;
    private static final int k_telescopeTolerance = 2500;
    private static final double k_telescopeMaxPower = 1.0;
    public static final int k_backMotorOffRatchetValue = 1000; // FIXME
    public static final double k_backOffMotorSpeed = -0.01; // FIXME
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

  public boolean m_telescopeBackOff = false;

  public RatchetPosition m_ratchetState = RatchetPosition.ENGAGED;
  public TelescopePosition m_telescopeState = TelescopePosition.RETRACTED;

  TalonFX m_telescopeMotor;
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
    m_telescopeMotor = new TalonFX(ExtensionConstants.k_extensionMotorID);
    m_ratchetServo = new Servo(ExtensionConstants.k_extensionServoID);
    m_telescopePIDController = new PIDController(ExtensionConstants.k_proportional, ExtensionConstants.k_integral,
        ExtensionConstants.k_derivative);
    m_telescopePIDController.setTolerance(ExtensionConstants.k_telescopeTolerance);
    // extensionServo.setBounds(1200, 125, 1100, 75, 1000);
  }

  @Override
  public void initialize() {
    resetEncoder();
    m_telescopeState = TelescopePosition.RETRACTED;
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
    m_telescopeMotor.setSelectedSensorPosition(0);
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
    if (inTolerance((int) m_telescopeMotor.getSelectedSensorPosition(), ExtensionConstants.k_retractedTelescopeValue,
        ExtensionConstants.k_telescopeTolerance)) {
      return TelescopePosition.RETRACTED;
    } else if (inTolerance((int) m_telescopeMotor.getSelectedSensorPosition(),
        ExtensionConstants.k_collectionTelescopeValue, ExtensionConstants.k_telescopeTolerance)) {
      return TelescopePosition.COLLECTION;
    } else if (inTolerance((int) m_telescopeMotor.getSelectedSensorPosition(),
        ExtensionConstants.k_lowPoleTelescopeValue, ExtensionConstants.k_telescopeTolerance)) {
      return TelescopePosition.LOW_POLE;
    } else if (inTolerance((int) m_telescopeMotor.getSelectedSensorPosition(),
        ExtensionConstants.k_highPoleTelescopeValue, ExtensionConstants.k_telescopeTolerance)) {
      return TelescopePosition.HIGH_POLE;
    } else {
      return TelescopePosition.TRANSITION;
    }
  }

  public double getTelescopeMotorPosition() {
    return m_telescopeMotor.getSelectedSensorPosition();
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
      case LOW_POLE:
        m_telescopeSetPoint = ExtensionConstants.k_lowPoleTelescopeValue;
        break;
      case HIGH_POLE:
        m_telescopeSetPoint = ExtensionConstants.k_highPoleTelescopeValue;
        break;
    }
  }

  private void updateWithPID() {
    m_telescopeMotor.setNeutralMode(NeutralMode.Brake);
    double k_MaxPower = 0;

    if (inTolerance((int) getTelescopeMotorPosition(), m_telescopeSetPoint, ExtensionConstants.k_telescopeTolerance)) {
      k_MaxPower = ExtensionConstants.k_telescopeMaxPower;
    } else {
      k_MaxPower = ExtensionConstants.k_telescopeMaxPower;
    }

    SmartDashboard.putNumber("k_MaxPower", k_MaxPower);

    m_telescopeMotor.set(
        TalonFXControlMode.PercentOutput,
        MathUtil.clamp(
            m_telescopePIDController.calculate(m_telescopeMotor.getSelectedSensorPosition(), m_telescopeSetPoint),
            -k_MaxPower, k_MaxPower));

  }

  @Override
  public void periodic() {
    if (m_enableExtensionPeriodic) {
      SmartDashboard.putNumber("Telescope Set Point", m_telescopeSetPoint);
      SmartDashboard.putNumber("Telescope Current Position", m_telescopeMotor.getSelectedSensorPosition());
      setRatchetState();
      setTelescopeState();
      updateWithPID();
    }

    


  }

  @Override
  public void simulationPeriodic() {

  }
}
