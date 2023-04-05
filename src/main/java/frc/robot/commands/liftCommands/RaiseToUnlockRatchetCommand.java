package frc.robot.commands.liftCommands;

import edu.wpi.first.wpilibj.Timer;
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
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        m_lift.m_state = LiftPosition.RATCHET;
    }

    @Override
    public boolean isFinished() {
        if(m_timer.get() > 0.1) {
            m_lift.m_state = LiftPosition.STOP;
            return true;
        } else {
            return false;
        }
    }

    
}
