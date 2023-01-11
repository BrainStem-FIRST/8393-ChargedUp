package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Grabber;

public class TeleopGrabber extends CommandBase {

    private Grabber grabber;
    private DoubleSupplier isRunning;

    public TeleopGrabber(Grabber grabber, DoubleSupplier isRunning) {
        this.grabber = grabber;
        this.isRunning = isRunning;
    }

    @Override
    public void execute() {
        grabber.stopGrabbr();
    }

}