package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Extension.RatchetPosition;

public class RatchetUnlockCommand extends CommandBase {
    
    private final Extension m_extension;
    private double m_startTime;
    private double SERVO_RATCHET_TIME = 5;

    public RatchetUnlockCommand(Extension extension) {
        m_extension = extension;
        addRequirements(m_extension);
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("Rachet Unlock Initialize", true);
        SmartDashboard.putBoolean("Rachet Unlock Execute", false); 
        SmartDashboard.putBoolean("Rachet Unlock Finished", false);
        m_startTime = Timer.getFPGATimestamp();
        
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("Rachet Unlock Execute", true);
    }

    @Override
    public boolean isFinished() {
        SmartDashboard.putNumber("Rachet Unlock Time", m_startTime);
        SmartDashboard.putNumber("Rachet Current Time", Timer.getFPGATimestamp());
        SmartDashboard.putNumber("Rachet Delta Time", Timer.getFPGATimestamp() - m_startTime);
        if ((Timer.getFPGATimestamp() - m_startTime) < SERVO_RATCHET_TIME) {
            SmartDashboard.putBoolean("Rachet Unlock Finished", false);
            return false;
        } else {
            SmartDashboard.putBoolean("Rachet Unlock Finished", true);
            return true;
        }
    }

    
}
