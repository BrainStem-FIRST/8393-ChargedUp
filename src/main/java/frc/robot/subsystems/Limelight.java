package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight extends SubsystemBase {

    public SmartDashboard table;

    public static final class LimeLightConstants {
    
    }

    public Limelight() {}

    public CommandBase exampleMethodCommand() {
      return runOnce(
          () -> {

          });
    }


      @Override
      public void periodic() {
    
      }
    
      @Override
      public void simulationPeriodic() {
    
      }


}
