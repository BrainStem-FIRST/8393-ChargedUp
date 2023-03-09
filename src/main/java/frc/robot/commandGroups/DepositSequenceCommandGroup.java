package frc.robot.commandGroups;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.collectorCommands.CollectorDepositCommand;
import frc.robot.commands.extensionCommands.ExtensionDepositSequenceCommand;
import frc.robot.commands.liftCommands.LiftDepositLowerCommand;
import frc.robot.commands.liftCommands.LiftLowerGroundCommand;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Extension.TelescopePosition;
import frc.robot.subsystems.Lift.LiftPosition;


public class DepositSequenceCommandGroup extends SequentialCommandGroup {
    Lift m_lift;
    DoubleSupplier m_triggerThreshold;
    Extension m_extension;
    Collector m_collector;
    TelescopePosition m_extensionPosition;
    
    public DepositSequenceCommandGroup(Lift p_lift, Extension p_extension, Collector p_collector, TelescopePosition p_extensionPosition) {
        this.m_lift = p_lift;
        this.m_extension = p_extension;
        this.m_collector = p_collector;
        this.m_extensionPosition = p_extensionPosition;
        addCommands(
            //new LiftDepositLowerCommand(m_lift),
            new CollectorDepositCommand(m_collector),
            new ExtensionDepositSequenceCommand(p_extension, m_extensionPosition),
            new LiftLowerGroundCommand(m_lift)
        );
        
    }

}