package frc.robot.commandGroups;

import java.time.Instant;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.collectorCommands.CollectorOpenCommand;
import frc.robot.commands.extensionCommands.BackMotorOffRatchetCommand;
import frc.robot.commands.extensionCommands.ExtensionCommand;
import frc.robot.commands.extensionCommands.RatchetLockCommand;
import frc.robot.commands.extensionCommands.RatchetUnlockCommand;
import frc.robot.commands.extensionCommands.RetractedCommand;
import frc.robot.commands.liftCommands.LiftDepositLowerCommand;
import frc.robot.commands.liftCommands.LiftGroundCommand;
import frc.robot.commands.liftCommands.LiftLowPoleCommand;
import frc.robot.commands.liftCommands.LiftCarryCommand;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Collector.CollectorConstants;
import frc.robot.subsystems.Collector.IntakeState;
import frc.robot.subsystems.Extension.TelescopePosition;
import frc.robot.subsystems.Lift.LiftConstants;
import frc.robot.subsystems.Lift.LiftPosition;

public class GroundCollectionCommandGroup extends SequentialCommandGroup {
    private Lift m_lift;
    public GroundCollectionCommandGroup(Extension extension, Lift p_lift, Collector collector) {
        m_lift = p_lift;
            addCommands(
                new InstantCommand(() -> collector.m_adjustableWheelMotorPower = CollectorConstants.k_wheelMotorSpeed * 1.25),
                 new LiftCarryCommand(p_lift)
                ,new InstantCommand(() -> extension.m_telescopeState = TelescopePosition.GROUND_COLLECTION)
                ,new InstantCommand(() -> m_lift.m_adjustableLiftSpeed = LiftConstants.k_MaxPower/2)
                ,new LiftGroundCommand(p_lift)
                ,new InstantCommand(() -> m_lift.m_adjustableLiftSpeed = LiftConstants.k_MaxPower)
                ,new CollectorOpenCommand(collector)
                ,new InstantCommand(() -> collector.m_intakeState = IntakeState.IN)
            );
        
      
    }

}
