package frc.robot.commands.extensionCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Extension.TelescopePosition;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.wpilibj.Timer;

public class ExtensionDepositSequenceCommand extends CommandBase {

    Timer z_timer = new Timer();

    private final Extension m_extension;
    TelescopePosition m_extensionPosition;
    boolean isFinished = false;

    public ExtensionDepositSequenceCommand(Extension p_extension, TelescopePosition p_extensionPosition) {
        m_extension = p_extension;
        this.m_extensionPosition = p_extensionPosition;
        addRequirements(m_extension);
    }

    @Override
    public void initialize() {
        z_timer.reset();
        z_timer.start();
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
        isFinished = true;
    }

    @Override
    public void execute(){
        SmartDashboard.putNumber("Lift Timer", z_timer.get());
    }

    @Override
    public boolean isFinished() {
        if(isFinished){
            isFinished = false;
            return true;
        } else {
            return false;
        }
        //return z_timer.get() > 1.0;
    }
}
