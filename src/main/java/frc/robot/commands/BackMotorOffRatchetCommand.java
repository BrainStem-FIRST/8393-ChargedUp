package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Extension.ExtensionConstants;
import frc.robot.subsystems.Extension.RatchetPosition;
import frc.robot.subsystems.Extension.TelescopePosition;

public class BackMotorOffRatchetCommand extends CommandBase {
    
    private final Extension m_extension;
    private double m_startTime;
    private double BACKOFF_TIME = 0.25;
    
    public BackMotorOffRatchetCommand(Extension extension) {
        m_extension = extension;
        addRequirements(m_extension);
    }

    @Override
    public void initialize() {
        m_startTime = Timer.getFPGATimestamp();
        m_extension.m_telescopeBackOff = true;
    }

    @Override
    public boolean isFinished() {
        if ((Timer.getFPGATimestamp() - m_startTime) > BACKOFF_TIME) {
            m_extension.m_telescopeBackOff = false;
            return true;
        } else {
            return false;
        }
    }
}
