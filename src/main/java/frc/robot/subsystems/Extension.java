package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commandGroups.AutoCubeCollectionExtensionCommandGroup;
import frc.robot.commandGroups.CollectionExtensionCommandGroup;
import frc.robot.commandGroups.HighPoleExtensionCommandGroup;
import frc.robot.commandGroups.LowPoleExtensionCommandGroup;
import frc.robot.commandGroups.RetractedExtensionCommandGroup;
import frc.robot.utilities.BrainSTEMSubsystem;

public class Extension extends SubsystemBase implements BrainSTEMSubsystem {

  public static final class ExtensionConstants {
    private static final double k_gearRatioMultiplication = 7.0 / 12.0;

    private static final int k_frontExtensionMotorID = 14;
    private static final int k_backExtensionMotorID = 31;
    private static final int k_extensionServoID = 9;
    private static final double k_proportional = 0.00009;
    private static final double k_integral = 0.00003;
    private static final double k_derivative = 0.00001;
    private static final int k_retractedTelescopeValue = 0; // 20000
    private static final int k_collectionTelescopeValue = (int) ((220000 / 1.8) * k_gearRatioMultiplication);
    private static final int k_lowPoleTelescopeValue = (int) ((int) (162000 * k_gearRatioMultiplication) * 1.02); // *1.1
    private static final int k_highPoleTelescopeValue = (int)(185508 * 0.965);
    private static final int k_telescopeTolerance = (int) (2000);
    private static final double k_telescopeMaxPower = 1.00; // 1.00
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
    AUTO_HIGH_POLE,
    TRANSITION,
    AUTO_CUBE_COLLECT
  }



  public double m_adjustableHighMultiplier = 1.0;
  public double m_adjustableLowMultiplier = 1.0;
  public double m_adjustableShelfCollectionMultiplier = 1.0;

  public double m_adjustableHighPoleTelescopeValue = ExtensionConstants.k_highPoleTelescopeValue * m_adjustableHighMultiplier;
  public double m_adjustableLowPoleTelescopeValue = ExtensionConstants.k_lowPoleTelescopeValue * m_adjustableLowMultiplier;
  public double m_adjustableShelfCollectionValue = ExtensionConstants.k_collectionTelescopeValue * m_adjustableShelfCollectionMultiplier;

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

    m_backMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 38.0, 38.0, 0.01));
    m_frontMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 38.0, 38.0, 0.01));

    m_backMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 38.0, 38.0, 0.01));
    m_frontMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 38.0, 38.0, 0.01));

  }

  @Override
  public void initialize() {
    m_backMotor.set(ControlMode.PercentOutput, 0);
    m_telescopeBackOff = false;
    m_telescopeSetPoint = 0;
    m_unlockPosition = 0;
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
    m_frontMotor.set(ControlMode.PercentOutput, 0);
  }

  public void ratchetDisengage() {
    m_ratchetServo.set(0.7);
  }

  public void ratchetEngage() {
    m_ratchetServo.set(0.6);
  }

  public void adjustExtensionValue(String point, double adjustmentAmount) {
    switch(point) {
      case "high": {
        m_adjustableHighMultiplier += adjustmentAmount;
        break;
      }
      case "low": {
        m_adjustableLowMultiplier += adjustmentAmount;
        break;
      }
      case "shelfCollection": {
        m_adjustableShelfCollectionValue += adjustmentAmount;
        break;
      } 
    }
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
      case AUTO_CUBE_COLLECT:
        new AutoCubeCollectionExtensionCommandGroup(this).schedule();
        break;
    }
  }

  public TelescopePosition getM_telescopeState() {
    if ((inTolerance((int) m_backMotor.getSelectedSensorPosition(), ExtensionConstants.k_retractedTelescopeValue,
        ExtensionConstants.k_telescopeTolerance)) && (m_telescopeState == TelescopePosition.RETRACTED)) {
      return TelescopePosition.RETRACTED;
    } else if ((inTolerance((int) m_backMotor.getSelectedSensorPosition(),
        (int)(m_adjustableShelfCollectionValue*m_adjustableShelfCollectionMultiplier), ExtensionConstants.k_telescopeTolerance))
        && (m_telescopeState == TelescopePosition.COLLECTION)) {
      return TelescopePosition.COLLECTION;
    } else if ((inTolerance((int) m_backMotor.getSelectedSensorPosition(),
        (int)(m_adjustableLowPoleTelescopeValue*m_adjustableLowMultiplier), ExtensionConstants.k_telescopeTolerance))
        && (m_telescopeState == TelescopePosition.LOW_POLE)) {
      return TelescopePosition.LOW_POLE;
    } else if ((inTolerance((int) m_backMotor.getSelectedSensorPosition(),
        (int)(m_adjustableHighPoleTelescopeValue*m_adjustableHighMultiplier), ExtensionConstants.k_telescopeTolerance))
        && (m_telescopeState == TelescopePosition.HIGH_POLE)) {
      return TelescopePosition.HIGH_POLE;
    } else if ((inTolerance((int) m_backMotor.getSelectedSensorPosition(),
        (int) (ExtensionConstants.k_highPoleTelescopeValue), ExtensionConstants.k_telescopeTolerance))
        && (m_telescopeState == TelescopePosition.AUTO_HIGH_POLE)) {
      return TelescopePosition.AUTO_HIGH_POLE;
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
        m_telescopeSetPoint = (int)(ExtensionConstants.k_collectionTelescopeValue*m_adjustableShelfCollectionMultiplier);
        break;
      case GROUND_COLLECTION:
        m_telescopeSetPoint = (int) (ExtensionConstants.k_collectionTelescopeValue * 0.7);
        break;
      case LOW_POLE:
        m_telescopeSetPoint = (int)(ExtensionConstants.k_lowPoleTelescopeValue*m_adjustableLowMultiplier);
        break;
      case HIGH_POLE:
        m_telescopeSetPoint = (int)(ExtensionConstants.k_highPoleTelescopeValue*m_adjustableHighMultiplier);
        break;
      case AUTO_CUBE_COLLECT:
        m_telescopeSetPoint = (int) (ExtensionConstants.k_lowPoleTelescopeValue * 1.3);
        break;
      case AUTO_HIGH_POLE:
        m_telescopeSetPoint = (int) (ExtensionConstants.k_highPoleTelescopeValue * 1.00);
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

  public CommandBase resetExtensionBase() {
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
      // m_frontMotor.setNeutralMode(NeutralMode.Coast);
      // m_backMotor.setNeutralMode(NeutralMode.Coast);

      // Extensoin TELEMETRY
      // ////////////////////////////////////////////////////////////////////////////////
      SmartDashboard.putNumber("E - Telescope Set Point", m_telescopeSetPoint);
      SmartDashboard.putNumber("E - Telescope Current Position", m_backMotor.getSelectedSensorPosition());
      SmartDashboard.putNumber("E - Extension amps", m_backMotor.getSupplyCurrent());
      SmartDashboard.putString("E - Returning State", getM_telescopeState().toString());
    }
  }

  public CommandBase disableBrakeModeBase() {
    return runOnce(() -> {
      m_backMotor.setNeutralMode(NeutralMode.Coast);
      m_frontMotor.setNeutralMode(NeutralMode.Coast);
    });
  }

  public CommandBase enableBrakeModeBase() {
    return runOnce(() -> {
      m_backMotor.setNeutralMode(NeutralMode.Brake);
      m_frontMotor.setNeutralMode(NeutralMode.Brake);
    });
  }

  @Override
  public void simulationPeriodic() {

  }
}
