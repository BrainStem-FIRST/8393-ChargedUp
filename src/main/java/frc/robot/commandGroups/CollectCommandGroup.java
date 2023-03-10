package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.collectorCommands.CollectorCloseCommand;
import frc.robot.commands.collectorCommands.IntakeOffCommand;
import frc.robot.commands.extensionCommands.BackMotorOffRatchetCommand;
import frc.robot.commands.extensionCommands.ExtensionCommand;
import frc.robot.commands.extensionCommands.RatchetLockCommand;
import frc.robot.commands.extensionCommands.RatchetUnlockCommand;
import frc.robot.commands.extensionCommands.RetractedCommand;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Extension.TelescopePosition;

public class CollectCommandGroup extends SequentialCommandGroup {
    
    public CollectCommandGroup(Collector collector) {
        addCommands(
            new CollectorCloseCommand(collector),
            new IntakeOffCommand(collector)
        );
      
    }

}
