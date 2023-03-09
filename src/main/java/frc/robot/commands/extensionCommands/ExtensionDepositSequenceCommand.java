package frc.robot.commands.extensionCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Extension.TelescopePosition;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.wpilibj.Timer;

public class ExtensionDepositSequenceCommand extends CommandBase {

    Timer m_timer = new Timer();

    private final Extension m_extension;
    TelescopePosition m_extensionPosition;

    public ExtensionDepositSequenceCommand(Extension p_extension, TelescopePosition p_extensionPosition) {
        m_extension = p_extension;
        this.m_extensionPosition = p_extensionPosition;
        addRequirements(m_extension);
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
        switch (m_extensionPosition) {
            case RETRACTED:
                m_extension.scheduleRetracted();
                break;
            case LOW_POLE:
                m_extension.scheduleLowPole();
                break;
            case HIGH_POLE:
                m_extension.scheduleHighPole();
                break;
            case COLLECTION:
                m_extension.scheduleCollection();
                break;
            default:
                m_extension.scheduleLowPole();
                break;
        }
    }

    @Override
    public void execute(){
        SmartDashboard.putNumber("Lift Timer", m_timer.get());
    }

    @Override
    public boolean isFinished() {
        return m_timer.get() > 1;
    }
}
