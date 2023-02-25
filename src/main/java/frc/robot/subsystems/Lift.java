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

  private static final class LiftConstants {
    private static final double k_P = 0.0005;
    private static final double k_I = 0.000;
    private static final double k_D = 0.000000;

    private static final int k_groundCollectionValue = 0;
    private static final int k_carryValue = 1;
    private static final int k_shelfCollectionValue = 3001;
    private static final int k_lowPoleValue = 3000;
    private static final int k_highPoleValue = 3400;
    private static final double k_MaxPower = 1.0;
  }

  private CANSparkMax m_forwardLift;
  private CANSparkMax mbackLift;
  private RelativeEncoder mliftForwardEncoder;
  private int m_liftSetPoint = 0;
  private boolean m_enableLiftPeriodic = false;
  PIDController m_liftPID;

  public Lift() {
    m_liftPID = new PIDController(LiftConstants.k_P, LiftConstants.k_I, LiftConstants.k_D);
    m_forwardLift = new CANSparkMax(2, MotorType.kBrushless);
    mbackLift = new CANSparkMax(3, MotorType.kBrushless);
    mliftForwardEncoder = m_forwardLift.getEncoder();
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
    HIGH_POLE
  }

  public LiftPosition m_state = LiftPosition.GROUND_COLLECTION;

  public boolean exampleCondition() {
    return false;
  }

  @Override
  public void initialize() {
    mliftForwardEncoder.setPosition(0);
    m_forwardLift.setIdleMode(IdleMode.kBrake);
    mbackLift.setIdleMode(IdleMode.kBrake);
    m_liftPID.setTolerance(15);
    mliftForwardEncoder.setPositionConversionFactor(42);
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
    mliftForwardEncoder.setPosition(0);
  }

  public void liftUp() {
    m_forwardLift.set(MathUtil.clamp(m_liftPID.calculate(mliftForwardEncoder.getPosition(), 3880), -0.05, 0.05));
    m_forwardLift.setIdleMode(IdleMode.kBrake);
    mbackLift.setIdleMode(IdleMode.kBrake);
    // mforwardLift.set(0.09);
  }

  public void liftUpIncs(double amount) {
    m_forwardLift.set(.1 * m_liftPID.calculate(mliftForwardEncoder.getPosition(), amount * 15));
  }

  public void liftDown() {
    m_forwardLift.setIdleMode(IdleMode.kCoast);
    mbackLift.setIdleMode(IdleMode.kCoast);
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
    m_state = LiftPosition.SHELF_COLLECTION;
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
        m_liftSetPoint = LiftConstants.k_lowPoleValue;
        break;
      case HIGH_POLE:
        m_liftSetPoint = LiftConstants.k_highPoleValue;
        break;
    }
  }

  private void updateWithPID() {
    m_forwardLift.setIdleMode(IdleMode.kBrake);
    SmartDashboard.putNumber("Lift Setpoint", m_liftSetPoint);
    SmartDashboard.putNumber("Lift PID Output",
        MathUtil.clamp(m_liftPID.calculate(mliftForwardEncoder.getPosition(), m_liftSetPoint),
            -LiftConstants.k_MaxPower, LiftConstants.k_MaxPower));
    SmartDashboard.putNumber("Lift Motor Encoder", mliftForwardEncoder.getPosition());

    // if (m_liftSetPoint > m_forwardLift.get()) {
    // if (((m_liftSetPoint - 90) < m_forwardLift.get()) && (m_forwardLift.get() <
    // m_liftSetPoint + 90)) {
    // m_forwardLift.set(0.001);
    // } else if ((m_liftSetPoint - m_forwardLift.get() > 250)) {
    // m_forwardLift.set(0.05);
    // } else {
    // m_forwardLift.set(MathUtil.clamp(mliftPID.calculate(mliftForwardEncoder.getPosition(),
    // m_liftSetPoint),
    // -LiftConstants.k_MaxPower, LiftConstants.k_MaxPower));
    // }
    // } else {
    // m_forwardLift.set(MathUtil.clamp(mliftPID.calculate(mliftForwardEncoder.getPosition(),
    // m_liftSetPoint),
    // -LiftConstants.k_MaxPower / 10, LiftConstants.k_MaxPower / 10));
    // }
    m_forwardLift.set(MathUtil.clamp(m_liftPID.calculate(mliftForwardEncoder.getPosition(), m_liftSetPoint),
        -LiftConstants.k_MaxPower, LiftConstants.k_MaxPower));
  }

  @Override
  public void periodic() {
    if (m_enableLiftPeriodic) {
      SmartDashboard.putBoolean("Lift Periodic Called", true);
      mbackLift.follow(m_forwardLift, true);
      setLiftState();
      updateWithPID();
    }
  }

  @Override
  public void simulationPeriodic() {

  }
}
