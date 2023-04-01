package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.BrainSTEMSubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Lift extends SubsystemBase implements BrainSTEMSubsystem {
  public static final class LiftConstants {
    public static final double k_P = 0.003;
    public static final double k_I = 0.0000;
    public static final double k_D = 0.0000;

    public static final int k_groundCollectionValue = 0;
    public static final int k_carryValue = 500;
    public static final int k_shelfCollectionValue = 3775;
    public static final int k_lowPoleValue = 3775;
    public static final int k_highPoleValue = 3560;
    public static final int k_highPoleTiltValue = 2800;
    public static final int k_liftPreLoadPosition = 300;
    public static final double k_MaxPower = 1.0;
    public static final int k_liftTolerance = 20;
    public static final int k_depositDelta = 400;

    public static final int k_hookServoID = 8;
    public static final double k_hookServoDownPosition = 0.99;
    public static final double k_hookServoUpPosition = 0.5;
    
  }

  private TalonFX m_forwardLift;
  private TalonFX m_backLift;
  private TalonFX m_rightLift;
  private Servo m_hookServo;
  private RelativeEncoder m_liftForwardEncoder;
  private RelativeEncoder m_liftBackEncoder;
  private int m_liftSetPoint = 0;
  private boolean m_enableLiftPeriodic = false;
  PIDController m_liftPID;
  public int depositDelta = 0;

  public Encoder m_liftEncoder;

  public double m_adjustableLiftSpeed = LiftConstants.k_MaxPower;

  public Lift() {

    m_liftPID = new PIDController(LiftConstants.k_P, LiftConstants.k_I, LiftConstants.k_D);

    m_forwardLift = new TalonFX(2);
    m_backLift = new TalonFX(3);
    m_rightLift = new TalonFX(0); //FIXME
    
    m_hookServo = new Servo(LiftConstants.k_hookServoID);

    m_liftEncoder = new Encoder(2, 5, false, EncodingType.k2X);

  }

  public CommandBase exampleMethodCommand() {
    return runOnce(
        () -> {

        });
  }

  public enum LiftPosition {
    GROUND_COLLECTION,
    CARRY,
    SHELF_COLLECTION,
    LOW_POLE,
    HIGH_POLE,
    HIGH_POLE_TILT,
    DEPOSIT_LOWER, 
    COLLECT_PRELOAD
  }

  public enum HookState {
    UP, 
    DOWN
  }

  public HookState m_hookState = HookState.UP;

  public LiftPosition m_state = LiftPosition.GROUND_COLLECTION;

  public boolean exampleCondition() {
    
    return false;
  
    
  }

  @Override
  public void initialize() {
    m_liftForwardEncoder.setPosition(0);
    m_liftBackEncoder.setPosition(0);
    m_liftPID.setTolerance(15);
    m_liftForwardEncoder.setPositionConversionFactor(42);
    m_liftBackEncoder.setPositionConversionFactor(42);
    m_state = LiftPosition.GROUND_COLLECTION;
    m_adjustableLiftSpeed = LiftConstants.k_MaxPower;
    m_liftSetPoint = 0;
    m_hookState = HookState.UP;
    m_hookServo.set(LiftConstants.k_hookServoUpPosition);
    m_liftEncoder.setDistancePerPulse(4.0/256);
    enablePeriodic();
  }

  @Override
  public void enablePeriodic() {
    m_enableLiftPeriodic = true;
  }

  @Override
  public void disablePeriodic() {
    m_enableLiftPeriodic = false;
  }

  public void resetLiftEncoder() {
    m_liftBackEncoder.setPosition(0);
  }

  public void liftUp() {
    m_forwardLift.set(ControlMode.PercentOutput, MathUtil.clamp(m_liftPID.calculate(m_liftBackEncoder.getPosition(), 3880), -0.05, 0.05));
    m_forwardLift.setNeutralMode(NeutralMode.Brake);
    m_backLift.setNeutralMode(NeutralMode.Brake);
  }

  public void turnOffLiftMotors() {
    m_forwardLift.set(ControlMode.PercentOutput, 0);
    m_backLift.set(ControlMode.PercentOutput, 0);
  }

  public void liftUpIncs(double amount) {
    m_forwardLift.set(ControlMode.PercentOutput, .1 * m_liftPID.calculate(m_liftBackEncoder.getPosition(), amount * 15));
  }

  public void liftDown() {
    m_forwardLift.setNeutralMode(NeutralMode.Coast);
    m_backLift.setNeutralMode(NeutralMode.Coast);
    m_rightLift.setNeutralMode(NeutralMode.Coast);
  }

  public double liftPosition() {
    return m_liftEncoder.getDistance();
  }

  
  

  public void liftStop() {
    m_liftPID.reset();
    m_forwardLift.set(ControlMode.PercentOutput, 0);
  }

  private boolean inTolerance(int currentPosition, int targetPosition, int tolerance) {
    return (currentPosition > (targetPosition - tolerance)) &&
        (currentPosition < (targetPosition + tolerance));
  }

  public void incrementLiftHeight() {
    m_liftPID.reset();

    if (m_state == LiftPosition.GROUND_COLLECTION) {
      m_state = LiftPosition.SHELF_COLLECTION;
    } else if (m_state == LiftPosition.SHELF_COLLECTION) {
      m_state = LiftPosition.LOW_POLE;
    } else if (m_state == LiftPosition.LOW_POLE) {
      m_state = LiftPosition.HIGH_POLE;
    } else {
      m_state = LiftPosition.GROUND_COLLECTION;
    }
  }

  public void decrementLiftHeight() {
    m_liftPID.reset();
    m_state = LiftPosition.GROUND_COLLECTION;
  }

  public void adjustLiftHeight(int ticks){
    m_liftSetPoint += ticks;
  }

  private void setHookState() {
    switch(m_hookState) {
      case UP:
      SmartDashboard.putString("HOOKS RN", "UP");
        m_hookServo.set(LiftConstants.k_hookServoUpPosition);
        break;
      case DOWN:
      SmartDashboard.putString("HOOKS RN", "DOWN");
        m_hookServo.set(LiftConstants.k_hookServoDownPosition);
        break;  
    }
  }

  private void setLiftState() {
    switch (m_state) {
      case GROUND_COLLECTION:
        m_liftSetPoint = LiftConstants.k_groundCollectionValue;
        break;
      case CARRY:
        m_liftSetPoint = LiftConstants.k_carryValue;
        break;
      case SHELF_COLLECTION:
        m_liftSetPoint = LiftConstants.k_shelfCollectionValue;
        break;
      case LOW_POLE:
        m_liftSetPoint = LiftConstants.k_lowPoleValue - depositDelta;
        break;
      case HIGH_POLE:
        m_liftSetPoint = LiftConstants.k_highPoleValue - depositDelta;
        break;
      case HIGH_POLE_TILT:
        m_liftSetPoint = LiftConstants.k_highPoleTiltValue;
        break;
      case DEPOSIT_LOWER:
        if(m_liftSetPoint != 0){
          m_liftSetPoint = m_liftSetPoint - 0; //200
        }
        break;
      case COLLECT_PRELOAD:
        m_liftSetPoint = LiftConstants.k_liftPreLoadPosition;
        break;
    }
  }

  private void updateWithPID() {
    m_forwardLift.setNeutralMode(NeutralMode.Brake);
    SmartDashboard.putNumber("Lift Setpoint", m_liftSetPoint);
    SmartDashboard.putNumber("Lift PID Output",
        MathUtil.clamp(m_liftPID.calculate(m_liftBackEncoder.getPosition(), m_liftSetPoint),
            -m_adjustableLiftSpeed, m_adjustableLiftSpeed));
    SmartDashboard.putNumber("Lift Motor Encoder", m_liftBackEncoder.getPosition());

    m_forwardLift.set(ControlMode.PercentOutput, MathUtil.clamp(m_liftPID.calculate(m_liftBackEncoder.getPosition(), m_liftSetPoint),
        -m_adjustableLiftSpeed, m_adjustableLiftSpeed));
  }

  public boolean isLiftAtCorrectPosition(){
    double liftPosition = m_liftBackEncoder.getPosition();
    return ((liftPosition < m_liftSetPoint + LiftConstants.k_liftTolerance) && (liftPosition > m_liftSetPoint - LiftConstants.k_liftTolerance));
  }

  @Override
  public void periodic() {
    if (m_enableLiftPeriodic) {
      SmartDashboard.putString("Lift Data", "------------------------------------");
      SmartDashboard.putNumber("Trough Bore Encoder", liftPosition());
      SmartDashboard.putString("Lift State", m_state.toString());
      SmartDashboard.putBoolean("Lift Periodic Called", true);
      SmartDashboard.getNumber("Hook Position set", m_hookServo.get());
      SmartDashboard.putNumber("Lift Back Encoder", m_liftBackEncoder.getPosition());
      m_backLift.follow(m_forwardLift);
      m_rightLift.follow(m_forwardLift);
      setLiftState();
      setHookState();
      updateWithPID();
    }
  }

  @Override
  public void simulationPeriodic() {

  }
}
