package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.WaitCommand;
import frc.robot.commands.extensionCommands.BackMotorOffRatchetCommand;
import frc.robot.commands.extensionCommands.ExtensionCommand;
import frc.robot.commands.extensionCommands.RatchetLockCommand;
import frc.robot.commands.extensionCommands.RatchetUnlockCommand;
import frc.robot.commands.extensionCommands.RetractedCommand;
import frc.robot.commands.liftCommands.HighPoleTiltCommand;
import frc.robot.commands.liftCommands.LiftDepositLowerCommand;
import frc.robot.commands.liftCommands.LiftHighPoleCommand;
import frc.robot.commands.liftCommands.LiftLowPoleCommand;
import frc.robot.commands.liftCommands.LowerHooksCommand;
import frc.robot.subsystems.NewCollector;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.NewCollector.CollectorConstants;
import frc.robot.subsystems.Extension.TelescopePosition;
import frc.robot.subsystems.Lift.LiftConstants;
import frc.robot.subsystems.Lift.LiftPosition;

public class HighPoleApproachCommandGroup extends SequentialCommandGroup {
    
    public HighPoleApproachCommandGroup(Extension extension, Lift lift, NewCollector collector) {
        addCommands(
            new LiftHighPoleCommand(lift),
            new LowerHooksCommand(lift),
            new WaitCommand(0.075),
            new HighPoleExtensionCommandGroup(extension),
            new InstantCommand(() -> lift.m_adjustableLiftSpeed = LiftConstants.k_MaxPower/2),
            new HighPoleTiltCommand(lift),
            new InstantCommand(() -> lift.m_adjustableLiftSpeed = LiftConstants.k_MaxPower)
        );
      
    }

}
