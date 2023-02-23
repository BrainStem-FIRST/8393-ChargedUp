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
  
    private static final double kclawMotorCurrentDrawLimit = 0.15;
    private static final double kclawMotorHoldingSpeed = 0;
    private static final double kwheelMotorSpeed = 0.1; //FIXME
    private static final double kwheelMotorCurrentDrawLimit = 1; //FIXME

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

  public CollectorState collectorState = CollectorState.OFF;
  public IntakeState intakeState = IntakeState.OFF;

  CANSparkMax clawMotor;
  CANSparkMax wheelMotor;
  RelativeEncoder clawMotorEncoder;
  PIDController clawMotorPIDController;

  private boolean clawButtonPressed = false;

  //double clawMotorSetPoint = CollectorConstants.clawOpenPosition;
  public Collector() {
    clawMotor = new CANSparkMax(CollectorConstants.kclawMotorID, MotorType.kBrushless);
    clawMotorEncoder = clawMotor.getEncoder();
    wheelMotor = new CANSparkMax(CollectorConstants.kwheelMotorID, MotorType.kBrushless);
    wheelMotor.setInverted(true); 
  }

  public void initialize() {
    clawMotorEncoder.setPosition(0);
    clawMotor.setIdleMode(IdleMode.kBrake);
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

  private void stopCollector() {
    clawMotor.stopMotor();
  }

  private void openCollector() {
    if (clawMotor.getOutputCurrent() < CollectorConstants.kclawMotorCurrentDrawLimit) {
      clawMotor.set(-0.02);
      SmartDashboard.putNumber("Collector Power", -0.02);
    } else {
      clawMotor.set(CollectorConstants.kclawMotorHoldingSpeed);
      SmartDashboard.putNumber("Collector Power", 0.0);
    }
  }

  private void closeCollector() {
    if (clawMotor.getOutputCurrent() < CollectorConstants.kclawMotorCurrentDrawLimit) {
      clawMotor.set(0.02);
      SmartDashboard.putNumber("Collector Power", 0.02);
    } else {
      clawMotor.set(CollectorConstants.kclawMotorHoldingSpeed);
      SmartDashboard.putNumber("Collector Power", 0.0);
    }
  }

  public void toggleClawButton() {
    clawButtonPressed = false;
  }

  public void toggleClawState() {
    if (!clawButtonPressed) {
      switch (collectorState) {
        case OPEN:
          collectorState = CollectorState.CLOSED;
          break;
        case CLOSED:
          collectorState = CollectorState.OPEN;
          break;
        case OFF:
          collectorState = CollectorState.OPEN;
          break;
      }
    } else {
      clawButtonPressed = true;
    }
  }

  private void setIntakeState() {
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
  }

  private void setCollectorState() {
    switch (collectorState) {
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
    setIntakeState();
    setCollectorState();


    SmartDashboard.putNumber("Collector Spinning Wheel Current Draw ", wheelMotor.getOutputCurrent());
    SmartDashboard.putNumber("Collector Claw Motor Current Draw", clawMotor.getOutputCurrent());
    
  }

  @Override
  public void simulationPeriodic() {
  
  }
}
