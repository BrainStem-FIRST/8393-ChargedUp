package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Extension.RatchetPosition;
import frc.robot.subsystems.Extension.TelescopePosition;

public class RetractedCommand extends CommandBase {
    
    private final Extension m_extension;
    private final TelescopePosition m_telescopePosition;

    public RetractedCommand(Extension extension) {
        m_extension = extension;
        this.m_telescopePosition = TelescopePosition.RETRACTED;
        addRequirements(m_extension);
    }

    @Override
    public boolean isFinished() {
        if (1 == 0) {
            return true;
        } else {
            return false;
        }
    }

    
}
