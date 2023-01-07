package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lift extends SubsystemBase {

    public static final class LiftConstants {
        private final static int LiftMotor1id = 13;
        private final static int LiftMotor2id = 14;
        private final static int LiftMotor3id = 15;
    }

    
    public Lift() {}

  public CommandBase exampleMethodCommand() {
    return runOnce(
        () -> {

        });
  }

  public boolean exampleCondition() {
    return false;
  }

  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {

  }
    
}
