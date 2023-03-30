package frc.robot.commands;

import frc.robot.subsystems.NewCollector;
import frc.robot.subsystems.Extension;

import frc.robot.subsystems.Extension.RatchetPosition;
import frc.robot.subsystems.Extension.TelescopePosition;
import frc.robot.subsystems.NewCollector.CollectorState;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoDepositCommand extends CommandBase {   

    private NewCollector m_collector;    
    private Extension m_extension;
    private Timer m_timer = new Timer();
    private int m_timeInSeconds;
    public AutoDepositCommand(NewCollector p_collector, Extension p_extension, int p_timeInSeconds) {

        this.m_collector = p_collector;
        this.m_extension = p_extension;
        this.m_timeInSeconds = p_timeInSeconds;
        addRequirements(m_collector, m_extension);

    }

    @Override
    public void initialize(){
        m_timer.reset();
        m_timer.start();
    }
    @Override
    public void execute() {
        if(m_timer.get() < 1.5){
            m_extension.m_telescopeState = TelescopePosition.COLLECTION;
        } else {
            m_collector.m_collectorState = CollectorState.DEPOSIT;
        }
    }

    @Override
    public boolean isFinished(){
        if(m_timer.get() > m_timeInSeconds){
            m_collector.setCollectorState();
            return true;
        } else {
            return false;
        }
        
    }
}
