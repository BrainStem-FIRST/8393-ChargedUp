package frc.robot.commands;

import java.util.function.BooleanSupplier;

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
    private BooleanSupplier autoMode;

    public DriveUntilLimelightCommand(boolean p_left, Swerve p_swerve, BooleanSupplier autoMode) {
        this.m_swerve = p_swerve;
        this.m_left = p_left;
        this.autoMode = autoMode;
        addRequirements(m_swerve);

        drivePIDController = new PIDController(0.01, 0,0);
    }

    @Override
    public void initialize() {
        isFinished = false;
    }

    @Override
    public void execute() {

        double strafePower = drivePIDController.calculate(LimelightHelpers.getTX("limelight-a"), 0);

        SmartDashboard.putBoolean("Limelight Get TV", LimelightHelpers.getTV("limelight-a"));

        if (LimelightHelpers.getTX("limelight-a") == 0.0) {
            if (this.m_left) {
                new TeleopSwerve(m_swerve, () -> 0, () -> 0.05, () -> 0, () -> false).schedule();
            } else {
                new TeleopSwerve(m_swerve, () -> 0, () -> -0.05, () -> 0, () -> false).schedule();
            }
        } else if ((LimelightHelpers.getTX("limelight-a") > 3) || (LimelightHelpers.getTX("limelight-a") < -3)){
            new TeleopSwerve(m_swerve, () -> 0, () -> strafePower, () -> 0, () -> false).schedule();
        } else {
            isFinished = true;
        }
        
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
