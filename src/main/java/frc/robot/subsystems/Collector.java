package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Collector extends SubsystemBase {
  
  private static final class CollectorConstants {
    private static final int k_clawMotorID = 19;
    private static final int k_wheelMotorID = 22;
  
    private static final double k_clawMotorCurrentDrawLimit = 0.15;
    private static final double k_clawMotorHoldingSpeed = 0;
    private static final double k_wheelMotorSpeed = 0.1; //FIXME
    private static final double k_wheelMotorCurrentDrawLimit = 3; //FIXME

  }

  public enum CollectorState {
    OPEN,
    CLOSED,
    OFF
  }

  public enum IntakeState {
    IN,
    OFF,
    OUT
  }

  public CollectorState m_collectorState = CollectorState.OFF;
  public IntakeState m_intakeState = IntakeState.OFF;

  CANSparkMax m_clawMotor;
  CANSparkMax m_wheelMotor;
  RelativeEncoder m_clawMotorEncoder;
  PIDController m_clawMotorPIDController;

  private boolean m_clawButtonPressed = false;
  private boolean m_intakeButtonPressed = false;

  //double clawMotorSetPoint = CollectorConstants.clawOpenPosition;
  public Collector() {
    m_clawMotor = new CANSparkMax(CollectorConstants.k_clawMotorID, MotorType.kBrushless);
    m_clawMotorEncoder = m_clawMotor.getEncoder();
    m_wheelMotor = new CANSparkMax(CollectorConstants.k_wheelMotorID, MotorType.kBrushless);
    m_wheelMotor.setInverted(true); 
  }

  public void initialize() {
    m_clawMotorEncoder.setPosition(0);
    m_clawMotor.setIdleMode(IdleMode.kBrake);
    m_clawMotor.set(0);
  }

  public void resetLiftEncoder(){
    m_clawMotorEncoder.setPosition(0);
  }

  public CommandBase exampleMethodCommand() {
    return runOnce(
        () -> {

        });
  }

  private void collectorIn() {
    SmartDashboard.putNumber("Collector Current Draw", m_wheelMotor.getOutputCurrent());
    if (m_wheelMotor.getOutputCurrent() < CollectorConstants.k_wheelMotorCurrentDrawLimit) {
      m_wheelMotor.set(CollectorConstants.k_wheelMotorSpeed);
    } else {
      m_intakeState = IntakeState.OFF; // FIXME set a tiny power
    }
  }

  private void collectorOut() {
    m_wheelMotor.set(-CollectorConstants.k_wheelMotorSpeed);
  }

  private void collectorOff() {
    m_wheelMotor.stopMotor();
  }

  private void stopCollector() {
    m_clawMotor.stopMotor();
  }

  private void openCollector() {
    if (m_clawMotor.getOutputCurrent() < CollectorConstants.k_clawMotorCurrentDrawLimit) {
      m_clawMotor.set(-0.02);
      SmartDashboard.putNumber("Collector Power", -0.02);
    } else {
      m_clawMotor.set(CollectorConstants.k_clawMotorHoldingSpeed);
      SmartDashboard.putNumber("Collector Power", 0.0);
    }
  }

  private void closeCollector() {
    if (m_clawMotor.getOutputCurrent() < CollectorConstants.k_clawMotorCurrentDrawLimit) {
      m_clawMotor.set(0.02);
      SmartDashboard.putNumber("Collector Power", 0.02);
    } else {
      m_clawMotor.set(CollectorConstants.k_clawMotorHoldingSpeed);
      SmartDashboard.putNumber("Collector Power", 0.0);
    }
  }

  public void toggleClawButton() {
    m_clawButtonPressed = false;
  }

  public void toggleClawState() {
    if (!m_clawButtonPressed) {
      switch (m_collectorState) {
        case OPEN:
          m_collectorState = CollectorState.CLOSED;
          break;
        case CLOSED:
          m_collectorState = CollectorState.OPEN;
          break;
        case OFF:
          m_collectorState = CollectorState.OPEN;
          break;
      }
    } else {
      m_clawButtonPressed = true;
    }
  }

  public void toggleIntakeState() {
    if (!m_intakeButtonPressed) {
      switch (m_intakeState) {
        case IN:
          m_intakeState = IntakeState.OFF;
          break;
        case OFF:
          m_intakeState = IntakeState.IN;
          break;
      }
    } else {
      m_intakeButtonPressed = true;
    }
  }

  public void toggleIntakeButton() {
    m_intakeButtonPressed = false;
  }


  private void setIntakeState() {
    switch (m_intakeState) {
      case IN:
        collectorIn();
        break;
      case OFF:
        collectorOff();
        break;
      case OUT:
        collectorOut();
        break;
    } 
  }

  private void setCollectorState() {
    switch (m_collectorState) {
      case OPEN:
        openCollector();
        break;
      case CLOSED:
        closeCollector();
        break;
      case OFF:
        stopCollector();
        break;
    }
  }

  @Override
  public void periodic() { //single responsibility principle so yea
    setIntakeState();
    setCollectorState();


    SmartDashboard.putNumber("Collector Spinning Wheel Current Draw ", m_wheelMotor.getOutputCurrent());
    SmartDashboard.putNumber("Collector Claw Motor Current Draw", m_clawMotor.getOutputCurrent());
    
  }

  @Override
  public void simulationPeriodic() {
  
  }
}
