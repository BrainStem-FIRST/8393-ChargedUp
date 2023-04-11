package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.utilities.BrainSTEMSubsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Collector extends SubsystemBase implements BrainSTEMSubsystem {

  public static final class CollectorConstants {
    public static final int k_wheelMotor2ID = 19;
    public static final int k_wheelMotorID = 22;
    public static final int k_clawDepositPosition = 509;
    public static final double k_clawMotorCurrentDrawLimit = 6.50;
    public static final double k_wheelMotorHoldingSpeed = 0.06;
    public static final double k_depositingSpeed = -0.35; // 0.4
    public static final double k_wheelMotorSpeed = 0.6; // FIXME
    public static final double k_wheelMotorCurrentDrawLimit = 40; // FIXME
    public static final double k_p = 0.0005; // FIXME
    public static final double k_i = 0; // FIXME
    public static final double k_d = 0; // FIXME

    public static final double k_feedForwardkS = 1;
    public static final double k_feedForwardkV = 12;
  }

  public enum CollectorState {
    OPEN,
    CLOSED,
    OFF,
    BEGINNING_AUTO
  }

  public enum IntakeState {
    IN,
    OFF,
    OUT,
    HOLD_IN
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

  private SimpleMotorFeedforward m_wheelMotorFeedForward;

  boolean firstTime = true;

  public boolean firstTIme = true;

  public boolean overLimit = false;

  public boolean objectCollected = false;

  public double m_adjustableWheelMotorPower = CollectorConstants.k_wheelMotorSpeed;

  public double m_adjustableWheelHoldingPower = CollectorConstants.k_wheelMotorHoldingSpeed;

  public boolean m_collectingByCommand = false;

  // double clawMotorSetPoint = CollectorConstants.clawOpenPosition;
  public Collector() {
    m_wheelMotor2 = new CANSparkMax(CollectorConstants.k_wheelMotor2ID, MotorType.kBrushless);
    m_clawMotorEncoder = m_wheelMotor2.getEncoder();
    m_wheelMotor = new CANSparkMax(CollectorConstants.k_wheelMotorID, MotorType.kBrushless);
    m_wheelMotor.setInverted(false); 
    m_wheelMotor2.setInverted(true);
    m_collectorPID = new PIDController(CollectorConstants.k_p, CollectorConstants.k_i, CollectorConstants.k_d);
    m_clawMotorEncoder.setPositionConversionFactor(42);

    m_wheelMotorFeedForward = new SimpleMotorFeedforward(CollectorConstants.k_feedForwardkS,
        CollectorConstants.k_feedForwardkV);
  }

  @Override
  public void initialize() {
    m_clawMotorEncoder.setPosition(0);
    m_wheelMotor2.setIdleMode(IdleMode.kBrake);
    m_wheelMotor2.set(0);
    m_wheelMotor.setIdleMode(IdleMode.kBrake);
    m_adjustableWheelMotorPower = CollectorConstants.k_wheelMotorSpeed;
    m_adjustableWheelHoldingPower = CollectorConstants.k_wheelMotorHoldingSpeed;
    m_wheelMotor.set(0);
    m_clawButtonPressed = false;
    m_intakeButtonPressed = false;
    m_collectorState = CollectorState.OFF;
    m_intakeState = IntakeState.OFF;
    enablePeriodic();
  }

  @Override
  public void enablePeriodic() {
    m_collectorPeriodicEnabled = true;
  }

  @Override
  public void disablePeriodic() {
    m_collectorPeriodicEnabled = false;
  }

  public void resetLiftEncoder() {
    m_clawMotorEncoder.setPosition(0);
  }

  public CommandBase exampleMethodCommand() {
    return runOnce(
        () -> {

        });
  }

  public CommandBase runOnceCommand() {
    return runOnce(
        () -> {
          resetCollectionMotor();
        });
  }

  public void resetCollectionMotor() {
    m_clawMotorEncoder.setPosition(0);
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

  public void resetCollectorTimer() {

    int i = 0;

    if (firstTIme) {
      i += 1;
      SmartDashboard.putNumber("A TEST", i);
      m_timer.reset();
      m_timer.start();
      firstTime = false;
    }

  }

  private void collectorIn() {
    double currentDraw = CollectorConstants.k_clawMotorCurrentDrawLimit;

    double speed = m_adjustableWheelMotorPower;

    // if (hasCollectorReachVelocity()) {
    // runOnceCommand();
    // }

    runOnceCommand();
    SmartDashboard.putBoolean("1AC - CONDITION ",
        ((m_wheelMotor2.getOutputCurrent() > 5 || (m_clawMotorEncoder.getVelocity() < 4500))
            && m_clawMotorEncoder.getPosition() > 2500));
    SmartDashboard.putBoolean("1AC - ENCODER ", m_clawMotorEncoder.getPosition() > 2500);
    SmartDashboard.putNumber("1AC - ENCODER Position", m_clawMotorEncoder.getPosition());
    SmartDashboard.putBoolean("1AC - Current Conditoin ",
        (m_wheelMotor2.getOutputCurrent() > 5 || (m_clawMotorEncoder.getVelocity() < 4500)));

    if((m_clawMotorEncoder.getVelocity() < 4500) && (m_clawMotorEncoder.getPosition() > 2500)) {
      collectorHold();
    } else {
      m_wheelMotor.set(m_adjustableWheelMotorPower);
      m_wheelMotor2.set(m_adjustableWheelMotorPower);
    }
    

  }

  private boolean hasCollectorReachVelocity() {
    return m_clawMotorEncoder.getVelocity() > 5000;
  }

  private void collectorOut() {
    m_wheelMotor.set(CollectorConstants.k_depositingSpeed * 1);
    m_wheelMotor2.set(CollectorConstants.k_depositingSpeed * 2.2);
    m_clawMotorEncoder.setPosition(0);
  }

  private void collectorHold() {
    m_wheelMotor.setIdleMode(IdleMode.kBrake);
    m_wheelMotor2.setIdleMode(IdleMode.kBrake);
    m_wheelMotor.set(m_adjustableWheelHoldingPower);
    m_wheelMotor2.set(m_adjustableWheelHoldingPower);
    m_clawMotorEncoder.setPosition(0);
  }

  private void stopCollector() {
    m_wheelMotor.setIdleMode(IdleMode.kBrake);
    m_wheelMotor2.setIdleMode(IdleMode.kBrake);
    m_wheelMotor.set(0.0);
    m_wheelMotor2.set(0.0);
  }

  public void stopCollectorAndIntake() {
    m_collectorState = CollectorState.OFF;
    m_intakeState = IntakeState.OFF;
  }

  private void openCollector() {
    // if (m_wheelMotor2.getOutputCurrent() <
    // CollectorConstants.k_clawMotorCurrentDrawLimit) {
    // m_wheelMotor2.set(m_adjustableClawMotorOpenPower);
    // } else {
    // m_wheelMotor2.set(CollectorConstants.k_clawMotorHoldingSpeed);
    // }
  }

  private void closeCollector() {
    // if (m_clawMotor.getOutputCurrent() <
    // CollectorConstants.k_clawMotorCurrentDrawLimit) {
    // m_clawMotor.set(CollectorConstants.k_clawMotorCloseSpeed);
    // } else {
    // m_clawMotor.set(CollectorConstants.k_clawMotorHoldingSpeed);
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
        stopCollector();
        break;
      case OUT:
        collectorOut();
        break;
      case HOLD_IN:
        collectorHold();
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

  public void updateWithFeedForward() {

    double feedForwardOutput = m_wheelMotorFeedForward.calculate(0.1);
    m_wheelMotor2.setVoltage(feedForwardOutput);
    m_wheelMotor.setVoltage(feedForwardOutput);
  }

  @Override
  public void periodic() {
    if (m_collectorPeriodicEnabled) {
      if (m_collectorState == CollectorState.BEGINNING_AUTO) {
        m_wheelMotor.set(0.1);
        m_wheelMotor2.set(0.1);
      } else {
        setIntakeState();
      }

      // updateWithFeedForward();

      // COLLECTOR TELEMETRY
      SmartDashboard.putNumber("C - Collector Wheel  Current Draw ", m_wheelMotor.getOutputCurrent());
      SmartDashboard.putNumber("C - Collector Wheel 2 Current Draw ", m_wheelMotor2.getOutputCurrent());
      SmartDashboard.putNumber("C - Collector Wheel 2 Encoder Position", m_clawMotorEncoder.getPosition());
      SmartDashboard.putNumber("C - COllector Wheel 2 Velocity ", m_clawMotorEncoder.getVelocity());

    }
  }

  @Override
  public void simulationPeriodic() {

  }
}