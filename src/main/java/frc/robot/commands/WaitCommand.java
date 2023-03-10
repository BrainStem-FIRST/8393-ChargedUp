package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class WaitCommand extends CommandBase {

    Timer m_timer = new Timer();
    Double time = 0.0;


    public WaitCommand(double Seconds) {
        time = Seconds;
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public boolean isFinished() {

        if (m_timer.get() > time) {
            return true;
        } else {
            return false;
        }

        
    } 
    
    
}
