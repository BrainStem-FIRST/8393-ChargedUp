package frc.robot.subsystems;

import com.ctre.phoenixpro.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Grabber extends SubsystemBase {

  public final static class GrabberConstants{
    public static final int grabberMotor1id = 18;
    public static final int grabberMotor2id = 19;
    public static final int grabberMotor3id = 20;
    public static final int grabberMotor4id = 21;

  }

  private TalonFX grabberMotor1 = new TalonFX(GrabberConstants.grabberMotor1id);
  private TalonFX grabberMotor2 = new TalonFX(GrabberConstants.grabberMotor2id);
  private TalonFX grabberMotor3 = new TalonFX(GrabberConstants.grabberMotor3id);
  private TalonFX grabberMotor4 = new TalonFX(GrabberConstants.grabberMotor4id);
  
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
    grabberMotor1.set(0.00);
    grabberMotor2.set(0.00);
    grabberMotor3.set(0.00);
    grabberMotor4.set(0.00);
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

