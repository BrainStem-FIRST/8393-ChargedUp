package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Extension.RatchetPosition;

public class RatchetUnlockCommand extends CommandBase {
    
    private final Extension m_extension;
    private double m_startTime;
    private double SERVO_RATCHET_TIME = 0.25;

    public RatchetUnlockCommand(Extension extension) {
        m_extension = extension;
        addRequirements(m_extension);
    }

    @Override
    public void initialize() {
        m_startTime = Timer.getFPGATimestamp();
        m_extension.m_ratchetState = RatchetPosition.DISENGAGED;
    }

    @Override
    public boolean isFinished() {
        return ((Timer.getFPGATimestamp() - m_startTime) > SERVO_RATCHET_TIME);
    }

    
}
