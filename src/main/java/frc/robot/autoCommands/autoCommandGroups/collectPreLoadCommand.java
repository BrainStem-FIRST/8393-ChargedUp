package frc.robot.autoCommands.autoCommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autoCommands.LiftCollectPreLoad;
import frc.robot.commands.collectorCommands.CollectorCloseCommand;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Lift;

public class CollectPreLoadCommand extends SequentialCommandGroup {
    

    public CollectPreLoadCommand(Lift lift, Collector collector) {
        addCommands(
            new LiftCollectPreLoad(lift),
            new CollectorCloseCommand(collector)
        );
        
    }


}
