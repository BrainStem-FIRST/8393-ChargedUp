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

  private static final class LiftConstants{
    private static final double kP = 0.001;
    private static final double kI = 0.001;
    private static final double kD = 0.000001;

    private static final int k_groundCollectionValue = 0;
    private static final int k_carryValue = 500;
    private static final int k_shelfCollectionValue = 1500;
    private static final int k_lowPoleValue = 1500;
    private static final int k_highPoleValue = 2000;
    private static final double k_MaxPower = 0.2;
  }

  private CANSparkMax mforwardLift;
  private CANSparkMax mbackLift;
  private RelativeEncoder mliftForwardEncoder;
  private int m_liftSetPoint = 0;

  PIDController mliftPID;

  public Lift() {
    mliftPID = new PIDController(LiftConstants.kP, LiftConstants.kI, LiftConstants.kD);
    mforwardLift = new CANSparkMax(2, MotorType.kBrushless);
    mbackLift = new CANSparkMax(3, MotorType.kBrushless);
    mliftForwardEncoder = mforwardLift.getEncoder();
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
    mforwardLift.setIdleMode(IdleMode.kBrake);
    mbackLift.setIdleMode(IdleMode.kBrake);
    mliftPID.setTolerance(15);
    mliftForwardEncoder.setPositionConversionFactor(42);
    mforwardLift.set(0);
  }

  public void resetLiftEncoder(){
    mliftForwardEncoder.setPosition(0);
  }
  

  public void liftUp() {
    mforwardLift.set(MathUtil.clamp(mliftPID.calculate(mliftForwardEncoder.getPosition(), 3880), -0.05, 0.05));
    mforwardLift.setIdleMode(IdleMode.kBrake);
    mbackLift.setIdleMode(IdleMode.kBrake);
    //mforwardLift.set(0.09);
  } 

  public void liftUpIncs(double amount) {
    mforwardLift.set(.1*mliftPID.calculate(mliftForwardEncoder.getPosition(), amount*15));
  }
  
  public void liftDown() {
    mforwardLift.setIdleMode(IdleMode.kCoast);
    mbackLift.setIdleMode(IdleMode.kCoast);
    //mforwardLift.set(.1*mliftPID.calculate(mliftForwardEncoder.getPosition(), 0));
  }

  public void liftStop() {
    mliftPID.reset();
    mforwardLift.set(0);
  }

  private boolean inTolerance(int currentPosition, int targetPosition, int tolerance) {
    return (currentPosition > (targetPosition - tolerance)) &&
      (currentPosition < (targetPosition + tolerance));
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

  private void updateWithPID(){
    mforwardLift.setIdleMode(IdleMode.kBrake);

    mforwardLift.set( 
      MathUtil.clamp(
        mliftPID.calculate(mliftForwardEncoder.getPosition(), m_liftSetPoint),
        -LiftConstants.k_MaxPower, LiftConstants.k_MaxPower
      )
    );
  }

  @Override
  public void periodic() {
    mbackLift.follow(mforwardLift, true);
    setLiftState();
    updateWithPID();
  }

  @Override
  public void simulationPeriodic() {

  }
}

