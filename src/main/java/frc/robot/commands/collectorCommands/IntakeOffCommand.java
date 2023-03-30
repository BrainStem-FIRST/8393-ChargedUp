package frc.robot.commands.collectorCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.NewCollector;
import frc.robot.subsystems.NewCollector.CollectorState;
import frc.robot.subsystems.NewCollector.IntakeState;

public class IntakeOffCommand extends CommandBase {
    
    private NewCollector collector; 
    private BooleanSupplier collectorOn;
    private BooleanSupplier closeCollector;
    Timer m_timer = new Timer();

    public IntakeOffCommand (NewCollector collector) {
        this.collector = collector;
        addRequirements(collector);
    }

    @Override
    public void initialize() {
        collector.m_collectorState = CollectorState.OFF;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
