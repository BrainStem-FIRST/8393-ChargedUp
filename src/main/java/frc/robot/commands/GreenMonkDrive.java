package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Swerve.SwerveConstants;
import frc.robot.utilities.LimelightHelpers;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class GreenMonkDrive extends CommandBase {
        public static final class MonkDriveContants {
                public static final double k_p = 0.1;
                public static final double k_i = 0;
                public static final double k_d = 0;
        }

        public enum LimelightMonkDirection {
                FORWARD,
                BACKWARD
        }


        private Swerve m_swerve;
        private BooleanSupplier robotCentricSup;
        private PIDController drivePIDController;
        private boolean m_targetLeft = true;
        private double strafePower = 0.05;


        public GreenMonkDrive(Swerve p_swerve, boolean p_targetLeft) {
                this.m_swerve = p_swerve;
                this.m_targetLeft = p_targetLeft;
                addRequirements(p_swerve);

                drivePIDController = new PIDController(MonkDriveContants.k_p, MonkDriveContants.k_i,
                                MonkDriveContants.k_d);
        }

        @Override
        public void execute() {
                if (!m_targetLeft) {
                        strafePower = strafePower * -1;
                }


                /* Drive */
                if (LimelightHelpers.getTV("limelight-a")) {
                        strafePower = drivePIDController.calculate(LimelightHelpers.getTX("limelight-a"), 0);
                } else {
                        strafePower = 0;
                }
                SmartDashboard.putNumber("Strafe Power", strafePower * 01);
                new TeleopSwerve(m_swerve, () -> 0, () -> strafePower * 0.1, () -> 0, () -> false).execute();
                
        }
}