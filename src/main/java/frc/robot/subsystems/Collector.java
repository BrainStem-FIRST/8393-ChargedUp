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
    private static final int kclawMotorID = 19;
    private static final int kwheelMotorID = 22;
    private static final int clawOpenPosition = 900; //FIXME
    private static final int clawClosePosition = 500; //FIXME
    private static final double PROPORTIONAL = 0.1; //FIXME
    private static final double INTEGRAL = 1; //FIXME
    private static final double DERIVATIVE = 0; //FIXME
    private static final double FEED_FORWARD = 0.000015; //FIXME
    private static final double kclawMotorCurrentDrawLimit = 1; //FIXME
    private static final double kclawMotorHoldingSpeed = 0; //FIXME
    private static final double kwheelMotorSpeed = 0.1; //FIXME
    private static final double kwheelMotorCurrentDrawLimit = 1; //FIXME
  

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
  //double clawMotorSetPoint = CollectorConstants.clawOpenPosition;
  public Collector() {
    clawMotor = new CANSparkMax(CollectorConstants.kclawMotorID, MotorType.kBrushless);
    clawMotorEncoder = clawMotor.getEncoder();
    wheelMotor = new CANSparkMax(CollectorConstants.kwheelMotorID, MotorType.kBrushless);
    clawMotorPIDController = new PIDController(CollectorConstants.PROPORTIONAL, CollectorConstants.INTEGRAL, CollectorConstants.DERIVATIVE);
    wheelMotor.setInverted(true); 
  }

  public void initialize() {
    clawMotorEncoder.setPosition(0);
    clawMotor.setIdleMode(IdleMode.kBrake);
    clawMotorPIDController.setTolerance(15);
    clawMotor.set(0);
  }

  public void resetLiftEncoder(){
    clawMotorEncoder.setPosition(0);
  }

  public CommandBase exampleMethodCommand() {
    return runOnce(
        () -> {

        });
  }

  private void collectorIn() {
    if (wheelMotor.getOutputCurrent() < CollectorConstants.kwheelMotorCurrentDrawLimit) {
      wheelMotor.set(CollectorConstants.kwheelMotorSpeed);
    } else {
      intakeState = IntakeState.OFF; // FIXME set a tiny power
    }
  }

  private void collectorOut() {
    wheelMotor.set(-CollectorConstants.kwheelMotorSpeed);
  }

  private void collectorOff() {
    wheelMotor.stopMotor();
  }

  private void openCollector() {
    clawMotor.set(0.02);
  }

  private void closeCollector() {
    if (clawMotor.getOutputCurrent() > CollectorConstants.kclawMotorCurrentDrawLimit) {
      clawMotor.set(0.02);
    } else {
      clawMotor.set(CollectorConstants.kclawMotorHoldingSpeed);
    }
  }

  @Override
  public void periodic() { //single responsibility principle so yea
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

    //clawMotor.set(clawMotorPIDController.calculate(clawMotorEncoder.getPosition(), clawMotorSetPoint));

    SmartDashboard.putNumber("Collector Spinning Wheel Current Draw ", wheelMotor.getOutputCurrent());
    SmartDashboard.putNumber("Collector Claw Motor Current Draw", clawMotor.getOutputCurrent());
    
  }

  @Override
  public void simulationPeriodic() {
  
  }
}
