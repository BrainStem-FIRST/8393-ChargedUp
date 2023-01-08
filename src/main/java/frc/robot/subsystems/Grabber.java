package frc.robot.subsystems;

import com.ctre.phoenixpro.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Grabber extends SubsystemBase {

  public static final class GrabberConstants {
   
    private final static int grabberMotor1 = 18; // FIXME 
    private final static int grabberMotor2 = 19; // FIXME 
    private final static int grabberMotor3 = 20; // FIXME 
    private final static int grabberMotor4 = 21; // FIXME 

  }

  public TalonFX grabberMotor1 = new TalonFX(GrabberConstants.grabberMotor1);
  public TalonFX grabberMotor2 = new TalonFX(GrabberConstants.grabberMotor2);
  public TalonFX grabberMotor3 = new TalonFX(GrabberConstants.grabberMotor3);
  public TalonFX grabberMotor4 = new TalonFX(GrabberConstants.grabberMotor4);

  public Grabber() {}

  public CommandBase exampleMethodCommand() {
    return runOnce(
        () -> {

        });
  }

  public void runGrabber() {
    grabberMotor1.set(0.65);
    grabberMotor2.set(0.65);
    grabberMotor3.set(0.65);
    grabberMotor4.set(0.65);
  }

  public void stopGrabbr() {
    grabberMotor1.set(0.0);
    grabberMotor2.set(0.0);
    grabberMotor3.set(0.0);
    grabberMotor4.set(0.0);
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

