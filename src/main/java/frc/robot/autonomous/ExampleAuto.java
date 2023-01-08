package frc.robot.autonomous;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.DrivetrainConstants;

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

public class ExampleAuto extends SequentialCommandGroup {
    private static final class ExampleAutoConstants {
        private static final double maxSpeedMetersPerSecond = 3;
        private static final double maxAccelerationMetersPerSecondSquared = 0; // FIXME
        private static final double proportionalThetaController = 0; // FIXME
        private static final double maxAngularSpeedRadiansPerSecond = Math.PI;
        private static final double maxAngularSpeedRadiansPerSecondSquared = Math.PI;
        private static final TrapezoidProfile.Constraints thetaControllerConstraints = new TrapezoidProfile.Constraints(
                maxAngularSpeedRadiansPerSecond, maxAngularSpeedRadiansPerSecondSquared);
        private static final double kPXController = 1;
        private static final double kPYController = 1;
        private static final double kPThetaController = 1;
    }

    public ExampleAuto(Drivetrain drivetrain) {
        TrajectoryConfig config = new TrajectoryConfig(
                ExampleAutoConstants.maxSpeedMetersPerSecond,
                ExampleAutoConstants.maxAccelerationMetersPerSecondSquared)
                .setKinematics(DrivetrainConstants.swerveKinematics);

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                config);

        var thetaController = new ProfiledPIDController(
                ExampleAutoConstants.proportionalThetaController, 0, 0,
                ExampleAutoConstants.thetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                exampleTrajectory,
                drivetrain::getPose,
                DrivetrainConstants.swerveKinematics,
                new PIDController(ExampleAutoConstants.kPXController, 0, 0),
                new PIDController(ExampleAutoConstants.kPYController, 0, 0),
                thetaController,
                drivetrain::setModuleStates,
                drivetrain);

        addCommands(
                new InstantCommand(() -> drivetrain.resetOdometry(exampleTrajectory.getInitialPose())),
                swerveControllerCommand);
    }
}