package frc.robot.commands.liftCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Lift.LiftPosition;

public class LiftCarryCommand extends CommandBase {
    
    private final Lift m_lift;

    public LiftCarryCommand(Lift p_lift) {
        m_lift = p_lift;
        addRequirements(m_lift);
    }

    @Override
    public void initialize() {
        m_lift.m_state = LiftPosition.CARRY;
    }

    @Override
    public boolean isFinished() {
        if(m_lift.isLiftAtCorrectPosition()) {
            Robot.depositSequenceContinue = true;
            return true;
        } else {
            return false;
        }
    }

    
}
