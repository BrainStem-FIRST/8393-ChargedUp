package frc.robot.commands.collectorCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Collector.CollectorState;
import edu.wpi.first.wpilibj.Timer;

public class CollectorDepositCommand extends CommandBase {
    
    private Collector m_collector; 
    Timer m_timer = new Timer();

    
    public CollectorDepositCommand (Collector p_collector) {
        this.m_collector = p_collector;
        addRequirements(m_collector);
    }

    @Override
    public void initialize(){
        m_timer.reset();
        m_collector.m_collectorState = CollectorState.OPEN;
        m_timer.start();
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished(){
        return m_timer.get() > 0.5;
    }


}
