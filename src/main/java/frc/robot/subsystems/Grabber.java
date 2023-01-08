package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Grabber extends SubsystemBase {
  public Grabber() {}

  public CommandBase exampleMethodCommand() {
    return runOnce(
        () -> {

        });
  }

<<<<<<< HEAD
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

=======
>>>>>>> 81ecbdaa23f80a830ef48315f5c85dffe41e7b93
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

