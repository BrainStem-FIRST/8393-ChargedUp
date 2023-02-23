package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Extension.RatchetPosition;
import frc.robot.subsystems.Extension.TelescopePosition;

public class ExtensionCommand extends CommandBase {
    
    private final Extension m_extension;
    private final TelescopePosition m_telescopePosition;
    private boolean stop = false;

    public ExtensionCommand(Extension extension, TelescopePosition telescopePosition) {
        m_extension = extension;
        this.m_telescopePosition = telescopePosition;
        addRequirements(m_extension);
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("Extension Command", true);
        m_extension.telescopeState = m_telescopePosition;
    }

    @Override
    public void execute() {
    
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    
}
