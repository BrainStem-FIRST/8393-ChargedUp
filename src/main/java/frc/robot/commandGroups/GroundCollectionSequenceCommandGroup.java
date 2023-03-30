package frc.robot.commandGroups;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.collectorCommands.CollectorDepositCommand;
import frc.robot.commands.extensionCommands.ExtensionDepositSequenceCommand;
import frc.robot.commands.liftCommands.LiftDepositLowerCommand;
import frc.robot.commands.liftCommands.LiftCarryCommand;
import frc.robot.subsystems.NewCollector;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Extension.TelescopePosition;
import frc.robot.subsystems.Lift.LiftConstants;
import frc.robot.subsystems.Lift.LiftPosition;


public class GroundCollectionSequenceCommandGroup extends SequentialCommandGroup {
    Lift m_lift;
    DoubleSupplier m_triggerThreshold;
    Extension m_extension;
    NewCollector m_collector;
    TelescopePosition m_extensionPosition;
    
    public GroundCollectionSequenceCommandGroup(Lift p_lift, Extension p_extension, NewCollector p_collector) {
        this.m_lift = p_lift;
        this.m_extension = p_extension;
        this.m_collector = p_collector;
        addCommands(
            new GroundCollectionCommandGroup(m_extension, m_lift, m_collector)
        );
        
    }

}