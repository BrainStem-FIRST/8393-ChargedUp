package frc.robot.commands.collectorCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.NewCollector;
import frc.robot.subsystems.NewCollector.CollectorState;
import frc.robot.subsystems.NewCollector.IntakeState;
import edu.wpi.first.wpilibj.Timer;

public class CollectorDepositCommand extends CommandBase {
    
    private NewCollector m_collector; 
    Timer m_timer = new Timer();

    
    public CollectorDepositCommand (NewCollector p_collector) {
        this.m_collector = p_collector;
        addRequirements(m_collector);
    }

    @Override
    public void initialize(){
        m_timer.reset();
        m_collector.m_intakeState = IntakeState.OUT;
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
