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
    
    public BackMotorOffRatchetCommand(Extension extension) {
        m_extension = extension;
        addRequirements(m_extension);
    }

    @Override
    public void initialize(){
        m_extension.m_unlockPosition = ExtensionConstants.k_backMotorOffRatchetValue;
    }

    @Override
    public boolean isFinished() {
        return m_extension.isBackedOff();
    }

    
}
