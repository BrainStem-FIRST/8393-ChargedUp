package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.BrainSTEMSubsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Lift extends SubsystemBase implements BrainSTEMSubsystem {
  public static final class LiftConstants {
    private static final double k_P = 0.003;
    private static final double k_I = 0.0000;
    private static final double k_D = 0.0000;

    private static final int k_groundCollectionValue = 0;
    private static final int k_carryValue = 500;
    private static final int k_shelfCollectionValue = 3800;
    private static final int k_lowPoleValue = 3700;
    private static final int k_highPoleValue = 3700;
    private static final int k_liftPreLoadPosition = 200;
    private static final double k_MaxPower = 1.0;
    private static final int k_liftTolerance = 50;
    public static final int k_depositDelta = 400;
  }

  private CANSparkMax m_forwardLift;
  private CANSparkMax m_backLift;
  private RelativeEncoder m_liftForwardEncoder;
  private int m_liftSetPoint = 0;
  private boolean m_enableLiftPeriodic = false;
  PIDController m_liftPID;
  public int depositDelta = 0;

  public Lift() {
    m_liftPID = new PIDController(LiftConstants.k_P, LiftConstants.k_I, LiftConstants.k_D);
    m_forwardLift = new CANSparkMax(2, MotorType.kBrushless);
    m_backLift = new CANSparkMax(3, MotorType.kBrushless);
    m_liftForwardEncoder = m_forwardLift.getEncoder();
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
    DEPOSIT_LOWER, 
    COLLECT_PRELOAD
  }

  public LiftPosition m_state = LiftPosition.GROUND_COLLECTION;

  public boolean exampleCondition() {
    return false;
  }

  @Override
  public void initialize() {
    m_liftForwardEncoder.setPosition(0);
    m_forwardLift.setIdleMode(IdleMode.kBrake);
    m_backLift.setIdleMode(IdleMode.kBrake);
    m_liftPID.setTolerance(15);
    m_liftForwardEncoder.setPositionConversionFactor(42);
    m_forwardLift.set(0);
    m_state = LiftPosition.GROUND_COLLECTION;
    m_liftSetPoint = 0;
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
    m_liftForwardEncoder.setPosition(0);
  }

  public void liftUp() {
    m_forwardLift.set(MathUtil.clamp(m_liftPID.calculate(m_liftForwardEncoder.getPosition(), 3880), -0.05, 0.05));
    m_forwardLift.setIdleMode(IdleMode.kBrake);
    m_backLift.setIdleMode(IdleMode.kBrake);
    // mforwardLift.set(0.09);
  }

  public void liftUpIncs(double amount) {
    m_forwardLift.set(.1 * m_liftPID.calculate(m_liftForwardEncoder.getPosition(), amount * 15));
  }

  public void liftDown() {
    m_forwardLift.setIdleMode(IdleMode.kCoast);
    m_backLift.setIdleMode(IdleMode.kCoast);
    // mforwardLift.set(.1*mliftPID.calculate(mliftForwardEncoder.getPosition(),
    // 0));
  }

  public void liftStop() {
    m_liftPID.reset();
    m_forwardLift.set(0);
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
        // switch (m_liftSetPoint) {
    //   case LiftConstants.k_groundCollectionValue:
    //     m_state = LiftPosition.CARRY;
    //     break;
    //   case LiftConstants.k_carryValue:
    //     m_state = LiftPosition.SHELF_COLLECTION;
    //     break;
    //   case LiftConstants.k_shelfCollectionValue:
    //     m_state = LiftPosition.LOW_POLE;
    //     break;
    //   case LiftConstants.k_lowPoleValue:
    //     m_state = LiftPosition.GROUND_COLLECTION;
    //     break;
    // }
    // m_state = LiftPosition.SHELF_COLLECTION;
  }

  public void decrementLiftHeight() {
    m_liftPID.reset();
    // switch (m_liftSetPoint) {
    //   case LiftConstants.k_groundCollectionValue:
    //     m_state = LiftPosition.LOW_POLE;
    //     break;
    //   case LiftConstants.k_carryValue:
    //     m_state = LiftPosition.GROUND_COLLECTION;
    //     break;
    //   case LiftConstants.k_shelfCollectionValue:
    //     m_state = LiftPosition.CARRY;
    //     break;
    //   case LiftConstants.k_lowPoleValue:
    //     m_state = LiftPosition.SHELF_COLLECTION;
    //     break;
    // }
    m_state = LiftPosition.GROUND_COLLECTION;
  }

  public void adjustLiftHeight(int ticks){
    m_liftSetPoint += ticks;
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
    m_forwardLift.setIdleMode(IdleMode.kBrake);
    SmartDashboard.putNumber("Lift Setpoint", m_liftSetPoint);
    SmartDashboard.putNumber("Lift PID Output",
        MathUtil.clamp(m_liftPID.calculate(m_liftForwardEncoder.getPosition(), m_liftSetPoint),
            -LiftConstants.k_MaxPower, LiftConstants.k_MaxPower));
    SmartDashboard.putNumber("Lift Motor Encoder", m_liftForwardEncoder.getPosition());

    m_forwardLift.set(MathUtil.clamp(m_liftPID.calculate(m_liftForwardEncoder.getPosition(), m_liftSetPoint),
        -LiftConstants.k_MaxPower, LiftConstants.k_MaxPower));
  }

  public boolean isLiftAtCorrectPosition(){
    double liftPosition = m_liftForwardEncoder.getPosition();
    return ((liftPosition < m_liftSetPoint + LiftConstants.k_liftTolerance) && (liftPosition > m_liftSetPoint - LiftConstants.k_liftTolerance));
  }

  @Override
  public void periodic() {
    if (m_enableLiftPeriodic) {
      SmartDashboard.putBoolean("Lift Periodic Called", true);
      m_backLift.follow(m_forwardLift, true);
      setLiftState();
      updateWithPID();
    }
  }

  @Override
  public void simulationPeriodic() {

  }
}
