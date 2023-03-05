package frc.robot.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Lift.LiftPosition;

public class LiftCollectPreLoad extends CommandBase {
    
    private final Lift m_lift;

    public LiftCollectPreLoad(Lift p_lift) {
        m_lift = p_lift;
        addRequirements(m_lift);
    }

    @Override
    public void initialize() {
        m_lift.m_state = LiftPosition.COLLECT_PRELOAD;
    }

    @Override
    public boolean isFinished() {
        return m_lift.isLiftAtCorrectPosition();
    }

    
}
