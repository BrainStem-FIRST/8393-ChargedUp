package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Extension;

public class TeleopExtension extends CommandBase {

    private Extension extension;
    private DoubleSupplier isRunning;

    
    public TeleopExtension(Extension extension, DoubleSupplier isRunning) {
        this.extension = extension;
        this.isRunning = isRunning;
    }

    @Override
    public void execute() {

    }
    
}
