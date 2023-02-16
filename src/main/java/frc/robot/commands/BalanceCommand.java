package frc.robot.commands;

import frc.robot.subsystems.Extension;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class BalanceCommand extends CommandBase {  
    private static final class BalanceCommandConstants {
        private static final double balanceProportional = 0.1; //FIXME
        private static final double balanceIntegral = 0.1; //FIXME
        private static final double balanceDerivative = 0.1; //FIXME
    } 

    private Extension extension;    
    private PIDController balancePIDController;
 
    public BalanceCommand(Extension extension)  {
        this.balancePIDController = new PIDController(BalanceCommandConstants.balanceProportional, 
        BalanceCommandConstants.balanceIntegral, BalanceCommandConstants.balanceDerivative);
        this.extension = extension;
        addRequirements(extension);
        
    }
    @Override
    public void execute() {
        
    }

}