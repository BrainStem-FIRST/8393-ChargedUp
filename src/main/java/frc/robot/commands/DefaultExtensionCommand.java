package frc.robot.commands;

import frc.robot.subsystems.Extension;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DefaultExtensionCommand extends CommandBase {   

    private Extension extension;    
    private DoubleSupplier extendDistance;
    private BooleanSupplier balance;
    public DefaultExtensionCommand(Extension extension, DoubleSupplier extendDistance, BooleanSupplier balance) {

        this.extension = extension;
        this.balance = balance;
        this.extendDistance = extendDistance;
        addRequirements(extension);

    }
    @Override
    public void execute() {
        
    }
}