package frc.robot.commands.liftCommands;

import edu.wpi.first.networktables.LogMessage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Lift.LiftPosition;

public class RaiseToUnlockRatchetCommand extends CommandBase {
    
    private final Lift m_lift;
    private Timer m_timer;

    public RaiseToUnlockRatchetCommand(Lift p_lift) {
        m_lift = p_lift;
        addRequirements(m_lift);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        m_lift.m_state = LiftPosition.RATCHET;
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
