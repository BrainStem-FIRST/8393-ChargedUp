package frc.robot.autoCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Swerve.SwerveConstants;
import frc.robot.utilities.LimelightHelpers;

public class DriveForwardUntilLimelight extends CommandBase {
    private Swerve m_swerve;

    public DriveForwardUntilLimelight(Swerve p_swerve) {
        this.m_swerve = p_swerve;
        addRequirements(m_swerve);

    }

    @Override
    public void execute() {
        m_swerve.drive(
                new Translation2d(0.2, 0).times(SwerveConstants.k_maxSpeed),
                0,
                false,
                true);
    }

    @Override
    public boolean isFinished() {
        if (LimelightHelpers.getTY("limelight-b") < -10) {
            new TeleopSwerve(m_swerve, () -> 0.0, () -> 0, () -> 0, () -> false).schedule();
            return true;
        } else {
            return false;
        }
    }

}
