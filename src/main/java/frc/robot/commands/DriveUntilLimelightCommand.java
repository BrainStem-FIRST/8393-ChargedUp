package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Swerve;
import frc.robot.utilities.LimelightHelpers;

public class DriveUntilLimelightCommand extends CommandBase {
    private boolean m_left;
    private Swerve m_swerve;
    private boolean isFinished = false;
    private int counter = 0;

    public DriveUntilLimelightCommand(boolean p_left, Swerve p_swerve) {
        this.m_swerve = p_swerve;
        this.m_left = p_left;
        addRequirements(m_swerve);
    }

    @Override
    public void initialize() {
        isFinished = false;
    }

    @Override
    public void execute() {
        
        if (!(LimelightHelpers.getTV("limelight"))) {
            if (this.m_left) {
                new TeleopSwerve(m_swerve, () -> 0, () -> 0.3, () -> 0, () -> false).schedule();
            } else {
                new TeleopSwerve(m_swerve, () -> 0, () -> -0.3, () -> 0, () -> false).schedule();
            }
        } else {
            new TeleopSwerve(m_swerve, () -> 0, () -> 0, () -> 0, () -> false).schedule();
            isFinished = true;
            
        }
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
