package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.collectorCommands.IntakeOffCommand;
import frc.robot.subsystems.NewCollector;


public class CollectCommandGroup extends SequentialCommandGroup {
    
    public CollectCommandGroup(NewCollector collector) {
        addCommands(
            new IntakeOffCommand(collector)
        );
      
    }

}
