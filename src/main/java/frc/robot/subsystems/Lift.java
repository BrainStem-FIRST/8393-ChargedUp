package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Lift extends SubsystemBase {
  public Lift() {}

  public CommandBase exampleMethodCommand() {
    return runOnce(
        () -> {

        });
  }

  public enum Position {
    UP,
    DOWN,
    STOP
  }
  private double kP = 0.000125;
  private double kI = 0.00053;
  private double kD = 0.000001;
  public TalonFX forwardLift = new TalonFX(2);
  PIDController pid = new PIDController(kP, kI, kD);

  public Position state = Position.UP;

  public boolean exampleCondition() {
    return false;
  }

  public void initialize() {
    forwardLift.setSelectedSensorPosition(0);
    forwardLift.setNeutralMode(NeutralMode.Brake);
    pid.setTolerance(50);
  }

  public void liftUp() {
    forwardLift.set(TalonFXControlMode.PercentOutput, pid.calculate(forwardLift.getSelectedSensorPosition(), 100000.0));
  } 
  
  public void liftDown() {
    forwardLift.set(TalonFXControlMode.PercentOutput, pid.calculate(forwardLift.getSelectedSensorPosition(), 0.0));
  }

  public void liftStop() {
    pid.reset();
    forwardLift.set(TalonFXControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
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
    SmartDashboard.putNumber("Forward Lift Motor Position", forwardLift.getSelectedSensorPosition());
    SmartDashboard.putNumber("Forward Lift Motor Power", pid.calculate(forwardLift.getSelectedSensorPosition()));
  }

  @Override
  public void simulationPeriodic() {

  }
}

