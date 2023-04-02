package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.utilities.BrainSTEMSubsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Collector extends SubsystemBase implements BrainSTEMSubsystem{
  
  public static final class CollectorConstants {
    public static final int k_wheelMotor2ID = 19;
    public static final int k_wheelMotorID = 22;
    public static final int k_clawDepositPosition = 509;
    public static final double k_clawMotorCurrentDrawLimit = 30;
    public static final double k_clawMotorHoldingSpeed = 0.20;
    public static final double k_clawMotorCloseSpeed = 0.20;
    public static final double k_clawMotorOpenSpeed = -0.03;
    public static final double k_wheelMotorSpeed = 0.2; //FIXME
    public static final double k_wheelMotorCurrentDrawLimit = 40; //FIXME
    public static final double k_p = 0.0005; //FIXME
    public static final double k_i = 0; //FIXME
    public static final double k_d = 0; //FIXME
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

  CANSparkMax m_wheelMotor2;
  CANSparkMax m_wheelMotor;
  RelativeEncoder m_clawMotorEncoder;
  PIDController m_clawMotorPIDController;

  private boolean m_clawButtonPressed = false;
  private boolean m_intakeButtonPressed = false;
  private boolean m_collectorPeriodicEnabled = false;
  private Timer m_timer = new Timer();
  PIDController m_collectorPID;

  public double m_adjustableClawMotorPower = CollectorConstants.k_clawMotorHoldingSpeed;

  public double m_adjustableWheelMotorPower = CollectorConstants.k_wheelMotorSpeed;

  public double m_adjustableClawMotorOpenPower = CollectorConstants.k_clawMotorOpenSpeed;

  //double clawMotorSetPoint = CollectorConstants.clawOpenPosition;
  public Collector() {
    m_wheelMotor2 = new CANSparkMax(CollectorConstants.k_wheelMotor2ID, MotorType.kBrushless);
    m_clawMotorEncoder = m_wheelMotor2.getEncoder();
    m_wheelMotor = new CANSparkMax(CollectorConstants.k_wheelMotorID, MotorType.kBrushless);
    m_wheelMotor.setInverted(true); 
    m_collectorPID = new PIDController(CollectorConstants.k_p, CollectorConstants.k_i, CollectorConstants.k_d);
    m_clawMotorEncoder.setPositionConversionFactor(42);
  }

  @Override
  public void initialize() {
    m_clawMotorEncoder.setPosition(0);
    m_wheelMotor2.setIdleMode(IdleMode.kBrake);
    m_wheelMotor2.set(0);
    m_wheelMotor.setIdleMode(IdleMode.kBrake);
    m_adjustableWheelMotorPower = CollectorConstants.k_wheelMotorSpeed;
    m_adjustableClawMotorPower = CollectorConstants.k_clawMotorHoldingSpeed;
    m_adjustableClawMotorOpenPower = CollectorConstants.k_clawMotorOpenSpeed;
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

  public void killCollectorMotors() {
    m_wheelMotor2.set(0);
    m_wheelMotor.set(0);
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
    m_timer.reset();
    m_wheelMotor.set(m_adjustableWheelMotorPower);
  }

  private void collectorOut() {
    m_wheelMotor.set(-m_adjustableWheelMotorPower);

  }

  private void collectorOff() {
    m_wheelMotor.setIdleMode(IdleMode.kBrake);
    m_wheelMotor.set(0);
    //m_wheelMotor.stopMotor();
  }

  private void stopCollector() {
    m_wheelMotor.setIdleMode(IdleMode.kBrake);
    m_wheelMotor.set(0);
    //m_clawMotor.stopMotor();
  }


  public void stopCollectorAndIntake(){
    m_collectorState = CollectorState.OFF;
    m_intakeState = IntakeState.OFF;
  }

  private void openCollector() {
    // if (m_wheelMotor2.getOutputCurrent() < CollectorConstants.k_clawMotorCurrentDrawLimit) {
    //   m_wheelMotor2.set(m_adjustableClawMotorOpenPower);
    // } else {
    //   m_wheelMotor2.set(CollectorConstants.k_clawMotorHoldingSpeed);
    // }
  }

  private void closeCollector() {
    // if (m_clawMotor.getOutputCurrent() < CollectorConstants.k_clawMotorCurrentDrawLimit) {
    //   m_clawMotor.set(CollectorConstants.k_clawMotorCloseSpeed);
    // } else {
    //   m_clawMotor.set(CollectorConstants.k_clawMotorHoldingSpeed);
    // }
    // m_wheelMotor2.set(m_adjustableClawMotorPower);
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
          m_collectorState = CollectorState.OFF;
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
      m_wheelMotor2.follow(m_wheelMotor, true);
      // setCollectorState();
      // SmartDashboard.putNumber("Collector Spinning Wheel Current Draw ", m_wheelMotor.getOutputCurrent());
      // SmartDashboard.putNumber("Collector Claw Motor Current Draw", m_clawMotor.getOutputCurrent());
      // SmartDashboard.putString("Intake State", m_intakeState.toString());
      // SmartDashboard.putNumber("Collector Claw Encoder Position", m_clawMotorEncoder.getPosition());
      // SmartDashboard.putNumber("Claw Speed", m_clawMotor.get());
    }
  }

  @Override
  public void simulationPeriodic() {
  
  }
}