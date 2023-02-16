package frc.robot.commands;

import frc.robot.subsystems.Tilt;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DefaultTiltCommand extends CommandBase {   

    private Tilt tilt;    
    private Supplier<Integer> tiltDistance;

    public DefaultTiltCommand(Tilt tilt, Supplier<Integer> tiltDistance) {

        this.tilt = tilt;
        this.tiltDistance = tiltDistance;
        addRequirements(tilt);

    }
    @Override
    public void execute() {
    
    }
}