package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.RetractedCommand;
import frc.robot.commands.BackMotorOffRatchetCommand;
import frc.robot.commands.ExtensionCommand;
import frc.robot.commands.RatchetLockCommand;
import frc.robot.commands.RatchetUnlockCommand;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Extension.TelescopePosition;

public class LowPoleExtensionCommandGroup extends SequentialCommandGroup {
    
    public LowPoleExtensionCommandGroup(Extension extension) {
        addCommands(
            new BackMotorOffRatchetCommand(extension),
            new RatchetUnlockCommand(extension),
            new ExtensionCommand(extension, TelescopePosition.LOW_POLE),
            new RatchetLockCommand(extension)
        );
      
    }

}
