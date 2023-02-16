package frc.robot.commands;

import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Lift;
import frc.robot.utilities.DepositHeights;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class DepositCommand extends CommandBase {
   

    private Collector collector;    
    private Extension extension;
    private Lift lift;
    private DepositHeights depositHeight;

    public DepositCommand(Collector collector, Extension extension, Lift lift, DepositHeights depositHeight) {
        this.collector = collector;
        this.extension = extension;
        this.lift = lift;
        this.depositHeight = depositHeight;
        addRequirements(collector, extension, lift);
    }

    @Override
    public void execute() {
        
    }
}