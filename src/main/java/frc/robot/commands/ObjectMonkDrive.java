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

public class ObjectMonkDrive extends CommandBase {
        public static final class CameraDriveMonkDriveContants {
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
        private double strafePower = 0.08;


        public ObjectMonkDrive(Swerve p_swerve, boolean p_targetLeft) {
                this.m_swerve = p_swerve;
                this.m_targetLeft = p_targetLeft;
                addRequirements(p_swerve);

                drivePIDController = new PIDController(CameraDriveMonkDriveContants.k_p, CameraDriveMonkDriveContants.k_i,
                CameraDriveMonkDriveContants.k_d);
        }

        @Override
        public void execute() {
                if (!m_targetLeft) {
                        strafePower = strafePower * -1;
                }


                /* Drive */
                if (LimelightHelpers.getTV("limelight-b")) {
                        strafePower = drivePIDController.calculate(LimelightHelpers.getTX("limelight-b"), 0);
                } 
                SmartDashboard.putNumber("111 - Strafe Power", strafePower * 0.5);
                SmartDashboard.putNumber("111 - Limelight TX ", LimelightHelpers.getTX("limelight-b"));
                new TeleopSwerve(m_swerve, () -> 0, () -> strafePower * 0.5, () -> 0, () -> false).execute();
                
        }
}