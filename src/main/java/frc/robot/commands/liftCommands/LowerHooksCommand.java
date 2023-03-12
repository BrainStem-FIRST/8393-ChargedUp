package frc.robot.commands.liftCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Lift.HookState;
import frc.robot.subsystems.Lift.LiftPosition;

public class LowerHooksCommand extends CommandBase {
    
    private final Lift m_lift;

    public LowerHooksCommand(Lift p_lift) {
        m_lift = p_lift;
        addRequirements(m_lift);
    }

    @Override
    public void initialize() {
        m_lift.m_hookState = HookState.DOWN;
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    
}
