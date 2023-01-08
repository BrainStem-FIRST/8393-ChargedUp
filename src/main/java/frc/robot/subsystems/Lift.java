package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lift extends SubsystemBase {

    public static final class LiftConstants {
        private final static int LiftMotor1id = 13; // FIXME 
        private final static int LiftMotor2id = 14; // FIXME 
        private final static int LiftMotor3id = 15; // FIXME 
    }

    
    public Lift() {}

  public CommandBase exampleMethodCommand() {
    return runOnce(
        () -> {

        });
  }

  public void runAllLiftMotors (double speed) {
    // run motors with speed
  }

  public void raiseLift (int height) {
    // raise the lift with a pid
  }

  public void lowerLift (int height) {
    // lowers the lift with a pid
  }

  public int liftHeight() {
    ruturn 1; 

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
