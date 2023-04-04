package frc.robot.commands.liftCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Lift.LiftPosition;

public class LiftLowPoleCommand extends CommandBase {
    
    private final Lift m_lift;

    public LiftLowPoleCommand(Lift p_lift) {
        m_lift = p_lift;
        addRequirements(m_lift);
    }

    @Override
    public void initialize() {
        m_lift.m_state = LiftPosition.LOW_POLE;
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    
}
