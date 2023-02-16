package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Collector extends SubsystemBase {
  private static final class CollectorConstants {
    private static final int clawMotorID = 26; //FIXME
    private static final int wheelMotorID = 71; //FIXME
    private static final double wheelMotorSpeed = 1; //FIXME
    private static final int clawOpenPosition = 900; //FIXME
    private static final int clawClosePosition = 500; //FIXME
    private static final double PROPORTIONAL = 0.1; //FIXME
    private static final double INTEGRAL = 1; //FIXME
    private static final double DERIVATIVE = 0; //FIXME
    private static final double FEED_FORWARD = 0.000015; //FIXME

  }

  
  
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
    
  }

  public CommandBase exampleMethodCommand() {
    return runOnce(
        () -> {

        });
  }

  private void collectorOn() {
    wheelMotor.set(CollectorConstants.wheelMotorSpeed);
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

  public void runCollector(boolean collectorOn, boolean closed) {
    if(collectorOn) {
      collectorOn();
    } else {
      collectorOff();
    }
    if(closed) {
      closeCollector();
    } else {
      openCollector();
    }
  }

  @Override
  public void periodic() {
    clawMotor.set(clawMotorPIDController.calculate(clawMotorEncoder.getPosition(), clawMotorSetPoint));
  }

  @Override
  public void simulationPeriodic() {
  
  }
}
