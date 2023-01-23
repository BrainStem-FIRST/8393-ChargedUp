package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Grabber extends SubsystemBase {
  public Grabber() {}

  public CommandBase exampleMethodCommand() {
    return runOnce(
        () -> {

        });
  }

  private CANSparkMax leftIntake = new CANSparkMax(26, MotorType.kBrushless);

  public boolean exampleCondition() {
    return false;
  }

  public void collectorOn() {
    leftIntake.set(0.5);
  }

  public void collectorOff() {
    leftIntake.set(0.0);
  }

  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {

  }
}

