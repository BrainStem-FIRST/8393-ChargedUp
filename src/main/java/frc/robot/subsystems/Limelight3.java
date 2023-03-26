package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.BrainSTEMSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class Limelight3 extends SubsystemBase implements BrainSTEMSubsystem {


    private double limeLightX;
    private double controllerDeadzone;
    private NetworkTableEntry limeLight;
    
    

    public static final class Limelight3Constants {
        

    }

    public Limelight3() {
        double limeLightDouble = limeLight.getDouble(0.0);
        this.limeLightX = limeLightDouble;
    }

    @Override
    public void initialize() {
        
        
    }

    @Override
    public void enablePeriodic() {
        
        
    }

    @Override
    public void disablePeriodic() {
        
        
    }

    @Override 
    public void periodic() {

        SmartDashboard.putNumber("Limelight Refelctive Tape", limeLightX);

    }
    
}
