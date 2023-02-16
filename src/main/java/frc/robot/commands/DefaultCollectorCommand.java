package frc.robot.commands;

import frc.robot.subsystems.Collector;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class DefaultCollectorCommand extends CommandBase {   

    private Collector collector;    
    private BooleanSupplier collectorOn;
    private BooleanSupplier closeCollector;

    public DefaultCollectorCommand(Collector collector, BooleanSupplier collectorOn, BooleanSupplier closeCollector) {

        this.collector = collector;
        this.collectorOn = collectorOn;
        this.closeCollector = closeCollector;
        addRequirements(collector);

    }
    @Override
    public void execute() {
        collector.runCollector(collectorOn.getAsBoolean(), closeCollector.getAsBoolean());
    }
}