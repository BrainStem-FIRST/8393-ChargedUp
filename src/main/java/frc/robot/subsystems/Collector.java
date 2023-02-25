package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.BrainSTEMSubsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Collector extends SubsystemBase implements BrainSTEMSubsystem{
  
  private static final class CollectorConstants {
    private static final int k_clawMotorID = 19;
    private static final int k_wheelMotorID = 22;
  
    private static final double k_clawMotorCurrentDrawLimit = 10;
    private static final double k_clawMotorHoldingSpeed = 0.01;
    private static final double k_clawMotorCloseSpeed = 0.05;
    private static final double k_clawMotorOpenSpeed = -0.05;
    private static final double k_wheelMotorSpeed = 0.3; //FIXME
    private static final double k_wheelMotorCurrentDrawLimit = 40; //FIXME

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
  private boolean m_collectorPeriodicEnabled = false;

  //double clawMotorSetPoint = CollectorConstants.clawOpenPosition;
  public Collector() {
    m_clawMotor = new CANSparkMax(CollectorConstants.k_clawMotorID, MotorType.kBrushless);
    m_clawMotorEncoder = m_clawMotor.getEncoder();
    m_wheelMotor = new CANSparkMax(CollectorConstants.k_wheelMotorID, MotorType.kBrushless);
    m_wheelMotor.setInverted(true); 
  }

  @Override
  public void initialize() {
    m_clawMotorEncoder.setPosition(0);
    m_clawMotor.setIdleMode(IdleMode.kBrake);
    m_clawMotor.set(0);
    m_wheelMotor.setIdleMode(IdleMode.kBrake);
    m_wheelMotor.set(0);
    m_clawButtonPressed = false;
    m_intakeButtonPressed = false;
    m_collectorState = CollectorState.OFF;
    m_intakeState = IntakeState.OFF; 
    enablePeriodic();
  }

  @Override
  public void enablePeriodic(){
    m_collectorPeriodicEnabled = true;
  }

  @Override
  public void disablePeriodic(){
    m_collectorPeriodicEnabled = false;
  }

  public void resetLiftEncoder(){
    m_clawMotorEncoder.setPosition(0);
  }

  public CommandBase exampleMethodCommand() {
    return runOnce(
        () -> {

        });
  }

  public void turnOnCollection() {
    m_intakeState = IntakeState.IN;
    m_collectorState = CollectorState.CLOSED;
  }

  public void turnOnDepositing() {
    m_intakeState = IntakeState.OUT;
    m_collectorState = CollectorState.OPEN;
  }

  public void turnOffCollection() {
    m_intakeState = IntakeState.OFF;
    m_collectorState = CollectorState.OFF;
  }

  private void collectorIn() {
    // SmartDashboard.putNumber("Collector Current Draw", m_wheelMotor.getOutputCurrent());
    // if (m_wheelMotor.getOutputCurrent() < CollectorConstants.k_wheelMotorCurrentDrawLimit) {
    //   m_wheelMotor.set(CollectorConstants.k_wheelMotorSpeed);
    // } else {
    //   m_intakeState = IntakeState.OFF;
    //   m_collectorState = CollectorState.CLOSED;
    // }
    m_wheelMotor.set(CollectorConstants.k_wheelMotorSpeed);
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
      m_clawMotor.set(CollectorConstants.k_clawMotorOpenSpeed);
    } else {
      m_clawMotor.set(CollectorConstants.k_clawMotorHoldingSpeed);
    }
  }

  private void closeCollector() {
    if (m_clawMotor.getOutputCurrent() < CollectorConstants.k_clawMotorCurrentDrawLimit) {
      m_clawMotor.set(CollectorConstants.k_clawMotorCloseSpeed);
    } else {
      m_clawMotor.set(CollectorConstants.k_clawMotorHoldingSpeed);
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
    if(m_collectorPeriodicEnabled){
      setIntakeState();
      setCollectorState();
      SmartDashboard.putNumber("Collector Spinning Wheel Current Draw ", m_wheelMotor.getOutputCurrent());
      SmartDashboard.putNumber("Collector Claw Motor Current Draw", m_clawMotor.getOutputCurrent());
      SmartDashboard.putString("Intake State", m_intakeState.toString());
    }
  }

  @Override
  public void simulationPeriodic() {
  
  }
}
