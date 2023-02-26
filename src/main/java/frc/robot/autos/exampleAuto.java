package frc.robot.autos;

import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Swerve.SwerveConstants;
import java.util.List;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class exampleAuto extends SequentialCommandGroup {

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double k_maxSpeedMetersPerSecond = 3;
        public static final double k_maxAccelerationMetersPerSecondSquared = 3;
        public static final double k_maxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double k_maxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double k_pXController = 1;
        public static final double k_pYController = 1;
        public static final double k_pThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints k_thetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                k_maxAngularSpeedRadiansPerSecond, k_maxAngularSpeedRadiansPerSecondSquared);
    }
    
    public exampleAuto(Swerve s_Swerve){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    AutoConstants.k_maxSpeedMetersPerSecond,
                    AutoConstants.k_maxAccelerationMetersPerSecondSquared)
                .setKinematics(SwerveConstants.k_swerveKinematics);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Set the origin at (5,0) facing the +X direction
                // Robot starts facing the poles
                new Pose2d(5, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(
                    new Translation2d(3, 0) //Just kind of a test thing to see if another waypooint fixes auto
                ),
                // End 5 meters behind ahead of where we started, rotating 180 degrees, now facing forward
                new Pose2d(0, 0, new Rotation2d(Math.PI)),
                config);

        var thetaController =
            new ProfiledPIDController(
                AutoConstants.k_pThetaController * 2, 0, 0, AutoConstants.k_thetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                exampleTrajectory,
                s_Swerve::getPose,
                SwerveConstants.k_swerveKinematics,
                new PIDController(AutoConstants.k_pXController, 0, 0),
                new PIDController(AutoConstants.k_pYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);


        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
            swerveControllerCommand
        );
    }
}