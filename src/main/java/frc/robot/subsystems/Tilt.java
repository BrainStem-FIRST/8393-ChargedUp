package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Tilt extends SubsystemBase {
  private static final class TiltConstants {

    private static final int solenoidID = 101; //FIXME
    private static final double k_upPosition = 1.0;
    private static final double k_closePosition = 0;
    private static final int k_hookServoID = 0;

  }


  Servo m_hooksServo;

  public Tilt() {
    //tiltPneumatics = new DoubleSolenoid(null, 0, 0);

    m_hooksServo = new Servo(TiltConstants.k_hookServoID);
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

  public void lowerHooks() {
      m_hooksServo.set(TiltConstants.k_closePosition);
  }

  public void raiseHooks() {
    m_hooksServo.set(TiltConstants.k_upPosition);
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
