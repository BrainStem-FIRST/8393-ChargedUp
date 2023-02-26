package frc.robot.commands.collectorCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Collector.CollectorState;

public class CollectorOpenCommand extends CommandBase {
    
    private Collector collector; 
    private BooleanSupplier collectorOn;
    private BooleanSupplier closeCollector;

    
    public CollectorOpenCommand (Collector collector, BooleanSupplier collectorOn, BooleanSupplier closeCollector) {

        this.collector = collector;
        this.collectorOn = collectorOn;
        this.closeCollector = closeCollector;
        addRequirements(collector);
    }

    @Override
    public void execute() {
        collector.m_collectorState = CollectorState.OPEN;
    }


}
