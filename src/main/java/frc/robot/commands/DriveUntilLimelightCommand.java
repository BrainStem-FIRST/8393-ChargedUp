package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
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
    private PIDController drivePIDController;

    public DriveUntilLimelightCommand(boolean p_left, Swerve p_swerve) {
        this.m_swerve = p_swerve;
        this.m_left = p_left;
        addRequirements(m_swerve);

        drivePIDController = new PIDController(0.1, 0,0);
    }

    @Override
    public void initialize() {
        isFinished = false;
    }

    @Override
    public void execute() {

        double strafePower = drivePIDController.calculate(LimelightHelpers.getTX("limelight-a"), 0);
        
        if (!(LimelightHelpers.getTV("limelight-a"))) {
            if (this.m_left) {
                new TeleopSwerve(m_swerve, () -> 0, () -> 0.17, () -> 0, () -> false).schedule();
            } else {
                new TeleopSwerve(m_swerve, () -> 0, () -> -0.17, () -> 0, () -> false).schedule();
            }
        } else {
            new TeleopSwerve(m_swerve, () -> 0, () -> strafePower, () -> 0, () -> false).schedule();
            isFinished = true;
            
        }
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
