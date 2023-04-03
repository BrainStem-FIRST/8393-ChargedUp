package frc.robot.commands.liftCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Lift.LiftPosition;

public class LiftShelfHeightCommand extends CommandBase {
    
    private final Lift m_lift;

    public LiftShelfHeightCommand(Lift p_lift) {
        m_lift = p_lift;
        addRequirements(m_lift);
    }

    @Override
    public void initialize() {
        m_lift.m_state = LiftPosition.SHELF_COLLECTION;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
