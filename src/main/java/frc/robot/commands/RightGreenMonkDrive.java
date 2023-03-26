package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;

public class RightGreenMonkDrive extends SequentialCommandGroup {
    private Swerve m_swerve;

    public RightGreenMonkDrive(Swerve p_swerve) {
        this.m_swerve = p_swerve;
        addCommands(
                new DriveUntilLimelightCommand(false, m_swerve),
                new GreenMonkDrive(m_swerve));
    }
}
