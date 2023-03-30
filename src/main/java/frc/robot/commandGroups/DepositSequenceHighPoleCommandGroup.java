package frc.robot.commandGroups;

import java.time.Instant;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.collectorCommands.CollectorDepositCommand;
import frc.robot.commands.extensionCommands.ExtensionDepositSequenceCommand;
import frc.robot.commands.liftCommands.LiftDepositLowerCommand;
import frc.robot.commands.liftCommands.LiftHighPoleCommand;
import frc.robot.commands.liftCommands.RaiseHooksCommand;
import frc.robot.commands.liftCommands.LiftCarryCommand;
import frc.robot.subsystems.NewCollector;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.NewCollector.CollectorConstants;
import frc.robot.subsystems.NewCollector.CollectorState;
import frc.robot.subsystems.NewCollector.IntakeState;
import frc.robot.subsystems.Extension.TelescopePosition;
import frc.robot.subsystems.Lift.LiftConstants;
import frc.robot.subsystems.Lift.LiftPosition;


public class DepositSequenceHighPoleCommandGroup extends SequentialCommandGroup {
    Lift m_lift;
    DoubleSupplier m_triggerThreshold;
    Extension m_extension;
    NewCollector m_collector;
    TelescopePosition m_extensionPosition;
    WaitCommand m_WaitCommand;
    
    public DepositSequenceHighPoleCommandGroup(Lift p_lift, Extension p_extension, NewCollector p_collector) {
        this.m_lift = p_lift;
        this.m_extension = p_extension;
        this.m_collector = p_collector;
        
        addCommands(
            new InstantCommand(() -> m_collector.m_intakeState = IntakeState.OUT),
            new WaitCommand(0.1),
            new CollectorDepositCommand(m_collector),
            new InstantCommand(() -> m_collector.m_collectorState = CollectorState.OFF),
            new CollectionExtensionCommandGroup(m_extension),
            new InstantCommand(() -> m_lift.m_adjustableLiftSpeed = LiftConstants.k_MaxPower/2),
            new LiftHighPoleCommand(m_lift),
            new InstantCommand(() -> m_lift.m_adjustableLiftSpeed = LiftConstants.k_MaxPower),
            new RetractedExtensionCommandGroup(m_extension),
            new WaitCommand(0.05),
            new RaiseHooksCommand(m_lift),
            new WaitCommand(0.05),
            new LiftCarryCommand(m_lift),
            new InstantCommand(() -> m_collector.m_collectorState = CollectorState.OFF)
        );
        
    }

}