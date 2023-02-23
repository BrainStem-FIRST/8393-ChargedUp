package frc.robot.commands.defaultCommands;

import frc.robot.subsystems.Lift;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DefaultLiftCommand extends CommandBase {   

    private Lift lift;    
    private Supplier<Integer> liftDistance;

    public DefaultLiftCommand(Lift lift, Supplier<Integer> liftDistance) {

        this.lift = lift;
        this.liftDistance = liftDistance;
        addRequirements(lift);

    }
    @Override
    public void execute() {
        //lift.runToPosition(liftDistance.get());
    }
}