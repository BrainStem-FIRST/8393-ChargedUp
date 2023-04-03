package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.BrainSTEMSubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Lift extends SubsystemBase implements BrainSTEMSubsystem {
  public static final class LiftConstants {
    public static final double k_P = 0.00003;
    public static final double k_I = 0.0000000;
    public static final double k_D = 0.0000;

    // All Volts 
    public static final double k_S = 0.0;
    public static final double k_G = 0.055; 
    public static final double k_V = 0.0000;

    public static final int k_forwardLiftMotorID = 30;
    public static final int k_backLiftMotorID = 33;
    public static final int k_rightLiftMotorID = 32;

    public static final int k_groundCollectionValue = 0;
    public static final int k_carryValue = Lift.inchesToTicks(1);
    public static final int k_shelfCollectionValue = Lift.inchesToTicks(17);
    public static final int k_lowPoleValue = Lift.inchesToTicks(15);
    public static final int k_highPoleValue = Lift.inchesToTicks(18);
    public static final int k_highPoleTiltValue = Lift.inchesToTicks(15);
    public static final int k_liftPreLoadPosition = Lift.inchesToTicks(15);
    public static final double k_MaxPower = 1.0;
    public static final int k_liftTolerance = Lift.inchesToTicks(0.09);
    public static final int k_depositDelta = 400 * 3000;

    public static final int k_hookServoID = 8;
    public static final double k_hookServoDownPosition = 0.99;
    public static final double k_hookServoUpPosition = 0.5;

  }

  TalonFX m_forwardLift;
  TalonFX m_backLift;
  TalonFX m_rightLift;
  private Servo m_hookServo;

  Encoder m_liftEncoder;

  private int m_liftSetPoint = 0;
  private boolean m_enableLiftPeriodic = false;
  PIDController m_liftPID;
  ElevatorFeedforward m_liftFeedForward;
  public int depositDelta = 0;

  public double m_adjustableLiftSpeed = LiftConstants.k_MaxPower;

  public Lift() {
    m_liftPID = new PIDController(LiftConstants.k_P, LiftConstants.k_I, LiftConstants.k_D);
    m_liftFeedForward = new ElevatorFeedforward(LiftConstants.k_S, LiftConstants.k_G, LiftConstants.k_V);
    m_forwardLift = new TalonFX(LiftConstants.k_forwardLiftMotorID);
    m_backLift = new TalonFX(LiftConstants.k_backLiftMotorID);
    m_rightLift = new TalonFX(LiftConstants.k_rightLiftMotorID);
    m_hookServo = new Servo(LiftConstants.k_hookServoID);

    m_forwardLift.configVoltageCompSaturation(12);
    m_backLift.configVoltageCompSaturation(12);
    m_rightLift.configVoltageCompSaturation(12);

    m_forwardLift.enableVoltageCompensation(true);
    m_backLift.enableVoltageCompensation(true);
    m_rightLift.enableVoltageCompensation(true);


    m_liftEncoder = new Encoder(5, 2);

    


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
    m_forwardLift.set(TalonFXControlMode.PercentOutput, 0);
    m_backLift.set(TalonFXControlMode.PercentOutput, 0);
    m_rightLift.set(TalonFXControlMode.PercentOutput, 0);

    m_forwardLift.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40.0, 40.0, 0.01));
    m_backLift.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40.0, 40.0, 0.01));
    m_rightLift.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40.0, 40.0, 0.01));

    m_forwardLift.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40.0, 40.0, 0.01));
    m_backLift.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40.0, 40.0, 0.01));
    m_rightLift.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40.0, 40.0, 0.01));

    m_forwardLift.configNeutralDeadband(0.01);
    m_backLift.configNeutralDeadband(0.01);
    m_rightLift.configNeutralDeadband(0.01);

    resetLiftEncoder();
    m_liftPID.setTolerance(45);

    m_state = LiftPosition.GROUND_COLLECTION;
    m_adjustableLiftSpeed = LiftConstants.k_MaxPower;
    m_liftSetPoint = 0;
    m_hookState = HookState.UP;
    m_hookServo.set(LiftConstants.k_hookServoUpPosition);
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
    m_forwardLift.setSelectedSensorPosition(0);
    m_backLift.setSelectedSensorPosition(0);
    m_rightLift.setSelectedSensorPosition(0);
  }

  public void liftUp() {
    m_forwardLift.set(TalonFXControlMode.PercentOutput,
        MathUtil.clamp(m_liftPID.calculate(m_forwardLift.getSelectedSensorPosition(), 3880), -0.05, 0.05));
    m_forwardLift.setNeutralMode(NeutralMode.Brake);
    m_backLift.setNeutralMode(NeutralMode.Brake);
    m_rightLift.setNeutralMode(NeutralMode.Brake);
  }

  public void liftRawPower(double power) {
    m_backLift.setInverted(InvertType.OpposeMaster);
    m_rightLift.setInverted(InvertType.OpposeMaster);
    m_backLift.follow(m_forwardLift);
    m_rightLift.follow(m_forwardLift);
    m_forwardLift.set(TalonFXControlMode.PercentOutput, -power);

  }

  public void turnOffLiftMotors() {
    m_forwardLift.set(TalonFXControlMode.PercentOutput, 0);
    m_backLift.set(TalonFXControlMode.PercentOutput, 0);
    m_rightLift.set(TalonFXControlMode.PercentOutput, 0);
  }

  public void liftUpIncs(double amount) {
    m_forwardLift.set(TalonFXControlMode.PercentOutput,
        .1 * m_liftPID.calculate(m_forwardLift.getSelectedSensorPosition(), amount * 15));
  }

  public void liftDown() {
    m_forwardLift.setNeutralMode(NeutralMode.Coast);
    m_backLift.setNeutralMode(NeutralMode.Coast);
    m_rightLift.setNeutralMode(NeutralMode.Coast);
    // mforwardLift.set(.1*mliftPID.calculate(mliftForwardEncoder.getPosition(),
    // 0));
  }

  // public void cycleHookServo(){
  // if(m_hookServo.get() == LiftConstants.k_hookServoUpPosition) {
  // m_hookServo.set(0.5);
  // SmartDashboard.putString("Hook Servo", "0.5");
  // } else if (m_hookServo.get() == 0.5) {
  // m_hookServo.set(LiftConstants.k_hookServoDownPosition);
  // SmartDashboard.putString("Hook Servo", "Hook Down Position");
  // } else if (m_hookServo.get() == LiftConstants.k_hookServoDownPosition) {
  // m_hookServo.set(LiftConstants.k_hookServoUpPosition);
  // SmartDashboard.putString("Hook Servo", "Hook Up Position");
  // }
  // }

  public static int inchesToTicks(double inches) {
    return (int) (((2048) / 1.75) * inches);
  }

  public void lowerHooks() {
    m_hookServo.set(LiftConstants.k_hookServoDownPosition);
  }

  public void raiseHooks() {
    m_hookServo.set(LiftConstants.k_hookServoUpPosition);
  }

  public void liftStop() {
    m_liftPID.reset();
    m_forwardLift.set(TalonFXControlMode.PercentOutput, 0);
  }

  public void liftBrake() {
    m_forwardLift.set(ControlMode.PercentOutput, -0.0);
    m_backLift.set(ControlMode.PercentOutput, -0.0);
    m_rightLift.set(ControlMode.PercentOutput, -0.0);

    m_forwardLift.setNeutralMode(NeutralMode.Brake);
    m_backLift.setNeutralMode(NeutralMode.Brake);
    m_rightLift.setNeutralMode(NeutralMode.Brake);
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

  public void lowerLiftToCarry() {
    m_state = LiftPosition.GROUND_COLLECTION;
  }

  public void decrementLiftHeight() {
    m_liftPID.reset();
    m_state = LiftPosition.GROUND_COLLECTION;
  }

  public void adjustLiftHeight(int ticks) {
    m_liftSetPoint += ticks;
  }

  private void setHookState() {
    switch (m_hookState) {
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
        if (m_liftSetPoint != 0) {
          m_liftSetPoint = m_liftSetPoint - 0; // 200
        }
        break;
      case COLLECT_PRELOAD:
        m_liftSetPoint = LiftConstants.k_liftPreLoadPosition;
        break;
    }
  }

  private void updateWithPID() {

    m_forwardLift.setNeutralMode(NeutralMode.Brake);
    m_backLift.setNeutralMode(NeutralMode.Brake);
    m_rightLift.setNeutralMode(NeutralMode.Brake);

    SmartDashboard.putNumber("Lift Setpoint", m_liftSetPoint);
    SmartDashboard.putNumber("Lift PID Output",
        MathUtil.clamp(m_liftPID.calculate(m_forwardLift.getSelectedSensorPosition(), m_liftSetPoint),
            -m_adjustableLiftSpeed, m_adjustableLiftSpeed));
    SmartDashboard.putNumber("Lift Motor Encoder", m_forwardLift.getSelectedSensorPosition());


    double liftPosition = m_forwardLift.getSelectedSensorPosition();

    // if (false) { //isLiftAtCorrectPosition()
    // m_forwardLift.set(TalonFXControlMode.PercentOutput, 0.03);
    // m_backLift.set(TalonFXControlMode.PercentOutput, -0.03);
    // m_rightLift.set(TalonFXControlMode.PercentOutput, -0.03);
    // } else {


    if (m_state == m_state.CARRY && liftPosition < 2000) {
      m_forwardLift.set(ControlMode.PercentOutput, 0);
      m_backLift.set(ControlMode.PercentOutput, 0);
      m_rightLift.set(ControlMode.PercentOutput, 0);
    } else if (liftPosition > m_liftSetPoint) {

      double motorPercentOutput = 0.3 * (MathUtil.clamp(m_liftPID.calculate(liftPosition, m_liftSetPoint),
      -m_adjustableLiftSpeed, m_adjustableLiftSpeed));
    
      double feedForwardOutput = m_liftFeedForward.calculate(m_liftSetPoint);

      m_forwardLift.set(TalonFXControlMode.PercentOutput,
      motorPercentOutput, DemandType.ArbitraryFeedForward, feedForwardOutput);

      m_backLift.set(TalonFXControlMode.PercentOutput,
      -motorPercentOutput, DemandType.ArbitraryFeedForward, -feedForwardOutput);

       m_rightLift.set(TalonFXControlMode.PercentOutput,
      -motorPercentOutput, DemandType.ArbitraryFeedForward, -feedForwardOutput);

      SmartDashboard.putNumber("Feed Forward for Lift ", feedForwardOutput);

    } else {
      double motorPercentOutput = MathUtil.clamp(m_liftPID.calculate(liftPosition, m_liftSetPoint),
      -m_adjustableLiftSpeed, m_adjustableLiftSpeed);
    
      double feedForwardOutput = m_liftFeedForward.calculate(m_liftSetPoint);

      m_forwardLift.set(TalonFXControlMode.PercentOutput,
      motorPercentOutput, DemandType.ArbitraryFeedForward, feedForwardOutput);

      m_backLift.set(TalonFXControlMode.PercentOutput,
      -motorPercentOutput, DemandType.ArbitraryFeedForward, -feedForwardOutput);

       m_rightLift.set(TalonFXControlMode.PercentOutput,
      -motorPercentOutput, DemandType.ArbitraryFeedForward, -feedForwardOutput);

      SmartDashboard.putNumber("Feed Forward for Lift ", feedForwardOutput);

    }

  }

  public boolean isLiftAtCorrectPosition() {
    double liftPosition = m_forwardLift.getSelectedSensorPosition();
    return ((liftPosition < m_liftSetPoint + LiftConstants.k_liftTolerance)
        && (liftPosition > m_liftSetPoint - LiftConstants.k_liftTolerance));
  }

  @Override
  public void periodic() {
    if (m_enableLiftPeriodic) {
      SmartDashboard.putString("Lift State", m_state.toString());
      SmartDashboard.putBoolean("Lift Periodic Called", true);
      SmartDashboard.getNumber("Hook Position set", m_hookServo.get());
      SmartDashboard.putNumber("Forward Lift Encoder", m_forwardLift.getSelectedSensorPosition());
      SmartDashboard.putNumber("Lift Back Encoder", m_backLift.getSelectedSensorPosition());
      
      setLiftState();
      SmartDashboard.putNumber("Front Motor Current",
          m_forwardLift.getSupplyCurrent());
      SmartDashboard.putNumber("Back Motor Current",
          m_backLift.getSupplyCurrent());
      SmartDashboard.putNumber("Right Motor Current",
          m_rightLift.getSupplyCurrent());
      SmartDashboard.putNumber("Front Motor Deprecated Current ",
          m_forwardLift.getOutputCurrent());
      setHookState();
      updateWithPID();

      // m_backLift.setNeutralMode(NeutralMode.Coast);
      // m_forwardLift.setNeutralMode(NeutralMode.Coast);
      // m_rightLift.setNeutralMode(NeutralMode.Coast);




      SmartDashboard.putNumber("Lift back Motor Encoder ", m_backLift.getSelectedSensorPosition());
      SmartDashboard.putNumber("Lift front Motor Encoder ", m_forwardLift.getSelectedSensorPosition());
      SmartDashboard.putNumber("Lift right Motor Encoder ", m_rightLift.getSelectedSensorPosition());
      SmartDashboard.putNumber("Lift Throughbore Encoder", m_backLift.getSelectedSensorPosition(1));
    }
  }

  @Override
  public void simulationPeriodic() {

  }
}