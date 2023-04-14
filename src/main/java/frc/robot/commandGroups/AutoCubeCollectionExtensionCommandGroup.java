package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.extensionCommands.BackMotorOffRatchetCommand;
import frc.robot.commands.extensionCommands.ExtensionCommand;
import frc.robot.commands.extensionCommands.RatchetLockCommand;
import frc.robot.commands.extensionCommands.RatchetUnlockCommand;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Extension.TelescopePosition;

public class AutoCubeCollectionExtensionCommandGroup extends SequentialCommandGroup {
    
    public AutoCubeCollectionExtensionCommandGroup(Extension extension) {

        addCommands(
            new ExtensionCommand(extension, TelescopePosition.AUTO_CUBE_COLLECT)
        );
       
    }

}
