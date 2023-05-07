package frc.robot.commands;

import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Swerve.SwerveConstants;
import frc.robot.utilities.LimelightHelpers;
import java.util.function.BooleanSupplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LimeLightDrive extends CommandBase {
        public static final class LimeLightDriveConstants {
                public static final double k_p = 0.1;
                public static final double k_autoP = 0.15;
                public static final double k_autoI = 0.001;
                public static final double k_i = 0;
                public static final double k_d = 0;
                public static final int k_limelightTolerance = 2;
        }

        public enum MonkDirection {
                FORWARD,
                BACKWARD
        }

        private Swerve m_swerve;
        private BooleanSupplier robotCentricSup;
        private PIDController drivePIDController;
        private PIDController autoPIDController;
        private boolean m_auto;
        private double currentYaw;
        private double rotationPower;

        public LimeLightDrive(Swerve p_swerve, boolean p_auto) {
                this.m_swerve = p_swerve;
                this.m_auto = p_auto;
                addRequirements(p_swerve);

                drivePIDController = new PIDController(LimeLightDriveConstants.k_p, LimeLightDriveConstants.k_i,
                LimeLightDriveConstants.k_d);

                autoPIDController = new PIDController(LimeLightDriveConstants.k_autoP, LimeLightDriveConstants.k_autoI,
                LimeLightDriveConstants.k_d);
        }

        @Override
        public void initialize() {
                currentYaw = 0;
                rotationPower = 0;

        }

        @Override
        public void execute() {
                if (!m_auto) {
                        currentYaw = m_swerve.getYaw().getDegrees();
                        rotationPower = drivePIDController.calculate(currentYaw, 180);

                        /* Drive */

                        m_swerve.drive(
                                        new Translation2d(0, 0),
                                        rotationPower * (SwerveConstants.k_maxAngularVelocity / 4),
                                        true,
                                        true);
                } else {
                        currentYaw = LimelightHelpers.getTX("limelight-a");
                        rotationPower = autoPIDController.calculate(currentYaw, -2);
                        
                        /* Drive */

                        m_swerve.drive(
                                        new Translation2d(0, 0),
                                        rotationPower * (SwerveConstants.k_maxAngularVelocity / 4),
                                        true,
                                        true);
                }
        }

        @Override
        public boolean isFinished() {
                if (m_auto && isObjectWithinTolerance()) {
                        m_swerve.drive(
                                        new Translation2d(0, 0),
                                        0,
                                        true,
                                        true);
                        return true;
                } else {
                        return false;
                }
        }

        private boolean isObjectWithinTolerance() {
                return ((LimelightHelpers.getTX("limelight-a") < (LimeLightDriveConstants.k_limelightTolerance - 2))
                                && (LimelightHelpers.getTX("limelight-a") > (-LimeLightDriveConstants.k_limelightTolerance - 2)));
        }
}