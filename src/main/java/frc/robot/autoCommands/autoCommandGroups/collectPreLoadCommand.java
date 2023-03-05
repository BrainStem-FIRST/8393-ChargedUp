package frc.robot.autoCommands.autoCommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autoCommands.LiftCollectPreLoad;
import frc.robot.commands.collectorCommands.CollectorCloseCommand;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Lift;

public class collectPreLoadCommand extends SequentialCommandGroup {
    

    public collectPreLoadCommand(Lift lift) {
        addCommands(
            new LiftCollectPreLoad(lift)
            
        );
        
    }


}
