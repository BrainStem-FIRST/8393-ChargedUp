package frc.robot.autoCommands.autoCommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autoCommands.LiftCollectPreLoad;
import frc.robot.subsystems.NewCollector;
import frc.robot.subsystems.Lift;

public class collectPreLoadCommand extends SequentialCommandGroup {
    

    public collectPreLoadCommand(Lift lift, NewCollector collector) {
        addCommands(
            new LiftCollectPreLoad(lift)
        );
        
    }


}
