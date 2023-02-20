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
  }

  private CANSparkMax mforwardLift;
  private CANSparkMax mbackLift;
  private RelativeEncoder mliftForwardEncoder;

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
    UP,
    DOWN,
    STOP
  }
  

  public LiftPosition state = LiftPosition.UP;

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

  @Override
  public void periodic() {
    mbackLift.follow(mforwardLift, true);
    switch (state) {
      case UP:
        liftUp();
        break;
      case STOP:
        liftStop();
        break;
      case DOWN:
        liftDown();
        break;
    }
    SmartDashboard.putNumber("Forward Lift Motor Position", mliftForwardEncoder.getPosition());
    SmartDashboard.putNumber("Forward Lift Motor Power", mforwardLift.get());
    SmartDashboard.putNumber("Back Lift Motor Power", mbackLift.get());
    SmartDashboard.putNumber("PID Output", mliftPID.calculate(mliftForwardEncoder.getPosition()));
  }

  @Override
  public void simulationPeriodic() {

  }
}

