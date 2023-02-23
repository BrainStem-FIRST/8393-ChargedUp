package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.RetractedCommand;
import frc.robot.commands.ExtensionCommand;
import frc.robot.commands.RatchetUnlockCommand;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Extension.TelescopePosition;

public class CollectionExtensionCommandGroup extends SequentialCommandGroup {
    
    public CollectionExtensionCommandGroup(Extension extension) {
        addCommands(
            new RatchetUnlockCommand(extension),
            new ExtensionCommand(extension, TelescopePosition.COLLECTION)
        );
        
    }

}
