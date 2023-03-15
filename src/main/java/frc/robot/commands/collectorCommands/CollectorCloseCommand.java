package frc.robot.commands.collectorCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Collector.CollectorState;
import frc.robot.subsystems.Collector.IntakeState;

public class CollectorCloseCommand extends CommandBase {
    
    private Collector collector; 
    private BooleanSupplier collectorOn;
    private BooleanSupplier closeCollector;
    Timer m_timer = new Timer();

    
    public CollectorCloseCommand (Collector collector) {
        this.collector = collector;
        addRequirements(collector);
    }

    @Override
    public void initialize() {
        m_timer.reset();
        collector.m_collectorState = CollectorState.CLOSED;
        m_timer.start();
    }

    @Override
    public boolean isFinished() {
        if (m_timer.get() > 0.75) {
            return true;
        } else {
            return false;
        }
    }


}
