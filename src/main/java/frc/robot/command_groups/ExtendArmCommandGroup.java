package frc.robot.command_groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ExtendTelescopeCommand;
import frc.robot.commands.RatchetUnlockCommand;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Extension.TelescopePosition;

public class ExtendArmCommandGroup extends SequentialCommandGroup {
    
    public ExtendArmCommandGroup(Extension extension, TelescopePosition telescopePosition) {
        addCommands(
            new RatchetUnlockCommand(extension),
            new ExtendTelescopeCommand(extension, telescopePosition)
        );
    }

}
