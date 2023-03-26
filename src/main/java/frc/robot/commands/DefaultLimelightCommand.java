package frc.robot.commands;

import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DefaultLimelightCommand extends CommandBase {
    private static final class Limelight3Constants {

    }

    private Limelight limelight;

    public DefaultLimelightCommand(Limelight limelight) {
        this.limelight = limelight;
        addRequirements(limelight);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}