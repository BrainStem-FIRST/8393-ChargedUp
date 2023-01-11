package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenixpro.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Extension extends SubsystemBase {

  public static final class ExtensionConstants {
    private final static int extensionMotorID = 16; //FIXME
    private final static int extensionABSencoderID = 1; //FIXME

  }

  private TalonFX extensionMotor = new TalonFX(ExtensionConstants.extensionABSencoderID);
  private CANCoder extensionEncoder = new CANCoder(ExtensionConstants.extensionABSencoderID);
  

  public Extension() {}

  public CommandBase exampleMethodCommand() {
    return runOnce(
        () -> {

        });
  }


  public void stopExtension() {
    extensionMotor.set(0.00);
  }

  public double extensionPosition(){
    return extensionEncoder.getPosition();
  }

  public void runExtension(double power){
    extensionMotor.set(power);
    
  }

  public void moveExtensionToHigherPosition(int position) {
    if (extensionPosition() < position) {
        runExtension(0.15);
    } else if (extensionPosition() < position - 300) {
      runExtension(0.25);
    } else {
      stopExtension();
    }
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

