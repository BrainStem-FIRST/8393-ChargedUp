package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Collector extends SubsystemBase {
  private static final class CollectorConstants {
    private static final int clawMotorID = 19;
    private static final int wheelMotorID = 22;
    private static final double wheelMotorSpeed = 0.1; //FIXME
    private static final int clawOpenPosition = 900; //FIXME
    private static final int clawClosePosition = 500; //FIXME
    private static final double PROPORTIONAL = 0.1; //FIXME
    private static final double INTEGRAL = 1; //FIXME
    private static final double DERIVATIVE = 0; //FIXME
    private static final double FEED_FORWARD = 0.000015; //FIXME

  }

  public enum CollectorState {
    OPEN,
    CLOSED
  }

  public enum IntakeState {
    IN,
    OFF,
    OUT
  }

  public CollectorState collectorState = CollectorState.CLOSED;
  public IntakeState intakeState = IntakeState.OFF;

  CANSparkMax clawMotor;
  CANSparkMax wheelMotor;
  RelativeEncoder clawMotorEncoder;
  PIDController clawMotorPIDController;
  double clawMotorSetPoint = CollectorConstants.clawOpenPosition;
  public Collector() {
    clawMotor = new CANSparkMax(CollectorConstants.clawMotorID, MotorType.kBrushless);
    clawMotorEncoder = clawMotor.getEncoder();
    wheelMotor = new CANSparkMax(CollectorConstants.wheelMotorID, MotorType.kBrushless);
    clawMotorPIDController = new PIDController(CollectorConstants.PROPORTIONAL, CollectorConstants.INTEGRAL, CollectorConstants.DERIVATIVE);
    wheelMotor.setInverted(true); 
  }

  public void initialize() {
    clawMotorEncoder.setPosition(0);
    clawMotor.setIdleMode(IdleMode.kBrake);
    m_clawPID.setTolerance(15);
    clawMotor.set(0);
  }

  public void resetLiftEncoder(){
    mliftForwardEncoder.setPosition(0);
  }

  public CommandBase exampleMethodCommand() {
    return runOnce(
        () -> {

        });
  }

  private void collectorIn() {
    wheelMotor.set(CollectorConstants.wheelMotorSpeed);
  }

  private void collectorOut() {
    wheelMotor.set(-CollectorConstants.wheelMotorSpeed);
  }

  private void collectorOff() {
    wheelMotor.stopMotor();
  }

  private void openCollector() {
    clawMotorSetPoint = CollectorConstants.clawOpenPosition;
  }

  private void closeCollector() {
    clawMotorSetPoint = CollectorConstants.clawClosePosition;
  }

  @Override
  public void periodic() {
    switch (intakeState) {
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

    switch (collectorState) {
      case OPEN:
        openCollector();
        break;
      case CLOSED:
        closeCollector();
        break;
    }

    clawMotor.set(clawMotorPIDController.calculate(clawMotorEncoder.getPosition(), clawMotorSetPoint));
  }

  @Override
  public void simulationPeriodic() {
  
  }
}
