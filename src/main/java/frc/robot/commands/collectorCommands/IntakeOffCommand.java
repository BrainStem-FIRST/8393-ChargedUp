package frc.robot.commands.collectorCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Collector.CollectorState;
import frc.robot.subsystems.Collector.IntakeState;

public class IntakeOffCommand extends CommandBase {
    
    private Collector collector; 
    private BooleanSupplier collectorOn;
    private BooleanSupplier closeCollector;
    Timer m_timer = new Timer();

    public IntakeOffCommand (Collector collector) {
        this.collector = collector;
        addRequirements(collector);
    }

    @Override
    public void initialize() {
        collector.m_intakeState = IntakeState.OFF;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
