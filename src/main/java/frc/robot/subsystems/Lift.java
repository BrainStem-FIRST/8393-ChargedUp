package frc.robot.subsystems;

import com.ctre.phoenixpro.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lift extends SubsystemBase {

    public static final class LiftConstants {

        private final static int LiftMotor1id = 13; // FIXME 
        private final static int LiftMotor2id = 14; // FIXME 
        private final static int LiftMotor3id = 15; // FIXME 
    
    }


    public TalonFX liftMotor1 = new TalonFX(LiftConstants.LiftMotor1id);
    public TalonFX liftMotor2 = new TalonFX(LiftConstants.LiftMotor2id);
    public TalonFX liftMotor3 = new TalonFX(LiftConstants.LiftMotor3id);

    
    public Lift() {}

  public CommandBase exampleMethodCommand() {
    return runOnce(
        () -> {

        });
  }

  public void runAllLiftMotors (double speed) {
    liftMotor1.set(speed);
    liftMotor2.set(speed);
    liftMotor3.set(speed);
  }

  public void raiseLift (int height) {
    // raise the lift with a pid (if needed)
  }

  public void lowerLift (int height) {
    // lowers the lift with a pid (if needed)
  }

  public void stopLift(){
    liftMotor1.set(0.00);
    liftMotor2.set(0.00);
    liftMotor3.set(0.00);
  }


  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {

  }
    
}
