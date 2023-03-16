package frc.robot.commandGroups;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.extensionCommands.BackMotorOffRatchetCommand;
import frc.robot.commands.extensionCommands.ExtensionCommand;
import frc.robot.commands.extensionCommands.RatchetLockCommand;
import frc.robot.commands.extensionCommands.RatchetUnlockCommand;
import frc.robot.commands.extensionCommands.RetractedCommand;
import frc.robot.commands.liftCommands.LiftDepositLowerCommand;
import frc.robot.commands.liftCommands.LiftLowPoleCommand;
import frc.robot.commands.liftCommands.LiftCarryCommand;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Collector.CollectorConstants;
import frc.robot.subsystems.Extension.TelescopePosition;
import frc.robot.subsystems.Lift.LiftPosition;

public class CarryRetractedCommandGroup extends SequentialCommandGroup {
    
    public CarryRetractedCommandGroup(Extension extension, Lift lift, Collector p_collector) {

        
        addCommands(
            new InstantCommand(() -> p_collector.m_adjustableClawMotorPower = CollectorConstants.k_clawMotorCloseSpeed),
            new LiftCarryCommand(lift),
            new RetractedExtensionCommandGroup(extension),
            new InstantCommand(() -> p_collector.m_adjustableClawMotorPower = CollectorConstants.k_clawMotorHoldingSpeed)
        );
      
    }

}
