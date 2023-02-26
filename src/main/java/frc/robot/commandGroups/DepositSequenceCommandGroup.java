package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.liftCommands.LiftDepositLowerCommand;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Lift;

public class DepositSequenceCommandGroup extends SequentialCommandGroup {
    Lift m_lift;
    
    public DepositSequenceCommandGroup(Lift p_lift, Extension p_extension) {
        this.m_lift = p_lift;
        addCommands(
            new LiftDepositLowerCommand(m_lift)
        );
        
    }

}
