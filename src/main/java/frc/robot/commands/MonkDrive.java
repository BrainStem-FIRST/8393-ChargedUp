package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Swerve.SwerveConstants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MonkDrive extends CommandBase {
        public static final class MonkDriveContants {
                public static final double k_p = 0.01;
                public static final double k_i = 0;
                public static final double k_d = 0;
        }

        public enum MonkDirection {
                FORWARD,
                BACKWARD
        }


        private Swerve m_swerve;
        private BooleanSupplier robotCentricSup;
        private PIDController drivePIDController;


        public MonkDrive(Swerve p_swerve) {
                this.m_swerve = p_swerve;
                addRequirements(p_swerve);

                drivePIDController = new PIDController(MonkDriveContants.k_p, MonkDriveContants.k_i,
                                MonkDriveContants.k_d);
        }

        @Override
        public void execute() {
                double currentYaw = m_swerve.getYaw().getDegrees();
                double rotationPower = drivePIDController.calculate(currentYaw, 180);


                /* Drive */
            
                        m_swerve.drive(
                                        new Translation2d(0, 0),
                                        rotationPower * SwerveConstants.k_maxAngularVelocity,
                                        true,
                                        true);
                
        }
}