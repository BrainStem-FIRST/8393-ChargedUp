package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Tilt extends SubsystemBase {
  private static final class TiltConstants {
  private static final int solenoidID = 101; //FIXME

  }

  DoubleSolenoid tiltPneumatics;
  public Tilt() {
    //tiltPneumatics = new DoubleSolenoid(null, 0, 0);
  }

  public CommandBase exampleMethodCommand() {
    return runOnce(
        () -> {

        });
  }


  
  public boolean exampleCondition() {
    return false;
  }

  public void setPosition(int Position){

  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
