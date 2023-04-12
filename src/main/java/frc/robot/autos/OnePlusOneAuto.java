package frc.robot.autos;

import frc.robot.autoCommands.LiftCollectPreLoad;
import frc.robot.autoCommands.autoCommandGroups.collectPreLoadCommand;
import frc.robot.commandGroups.AutoGroundCollectionCommandGroup;
import frc.robot.commandGroups.DepositSequenceCommandGroup;
import frc.robot.commandGroups.GroundCollectionCommandGroup;
import frc.robot.commandGroups.HighPoleApproachCommandGroup;
import frc.robot.commandGroups.LowPoleApproachCommandGroup;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.DriveUntilLimelightCommand;
import frc.robot.commands.GreenMonkDrive;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.collectorCommands.IntakeInCommand;
import frc.robot.commands.collectorCommands.IntakeOffCommand;
import frc.robot.commands.extensionCommands.ExtensionCommand;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Extension.TelescopePosition;
import frc.robot.subsystems.Swerve.SwerveConstants;
import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj.Timer;

public class OnePlusOneAuto extends SequentialCommandGroup {

    public static final class AutoConstants { // TODO: The below constants are used in the example auto, and must be
                                              // tuned to specific robot
        public static final double k_maxSpeedMetersPerSecond = 3;
        public static final double k_maxAccelerationMetersPerSecondSquared = 2;
        public static final double k_maxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double k_maxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double k_pXController = 1;
        public static final double k_pYController = 1;
        public static final double k_pThetaController = 2;

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints k_thetaControllerConstraints = new TrapezoidProfile.Constraints(
                k_maxAngularSpeedRadiansPerSecond, k_maxAngularSpeedRadiansPerSecondSquared);
    }

    private static final class BalanceConstants {
        private static final double k_P = 0.8;
        private static final double k_I = 0.000;
        private static final double k_D = 0.0;
    }

    public OnePlusOneAuto(Swerve s_Swerve, Lift m_Lift, Collector m_collector, Extension m_extension) {
        collectPreLoadCommand m_collectPreLoadCommand = new collectPreLoadCommand(m_Lift, m_collector);
        ExtensionCommand m_extensionCarry = new ExtensionCommand(m_extension, TelescopePosition.COLLECTION);
        LowPoleApproachCommandGroup m_lowPoleApproach = new LowPoleApproachCommandGroup(m_extension, m_Lift,
                m_collector);
        HighPoleApproachCommandGroup m_highPoleApproach = new HighPoleApproachCommandGroup(m_extension, m_Lift,
                m_collector);
        DepositSequenceCommandGroup m_depositSequenceCommandGroup = new DepositSequenceCommandGroup(m_Lift, m_extension,
                m_collector);
        IntakeOffCommand m_intakeOff = new IntakeOffCommand(m_collector);
        IntakeInCommand m_intakeIn = new IntakeInCommand(m_collector);
        GroundCollectionCommandGroup m_groundCollection = new GroundCollectionCommandGroup(m_extension, m_Lift,
                m_collector);
        DriveUntilLimelightCommand m_driveWithLimelightRIGHT = new DriveUntilLimelightCommand(false, s_Swerve, () -> true);
        Timer m_Timer = new Timer();

        TrajectoryConfig config = new TrajectoryConfig(
                AutoConstants.k_maxSpeedMetersPerSecond,
                AutoConstants.k_maxAccelerationMetersPerSecondSquared)
                .setKinematics(SwerveConstants.k_swerveKinematics);

        config.setReversed(true);
        // An example trajectory to follow. All units in meters.
        Trajectory goOutToCollectTrajectory = TrajectoryGenerator.generateTrajectory(
                // Set the origin at (5,0) facing the +X direction
                // Robot starts facing the poles
                new Pose2d(5.3, 0, new Rotation2d(Math.toRadians(0))),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(
                        new Translation2d(4, 0.25) // Just kind of a test thing to see if another waypooint fixes auto
                ),
                // End 5 meters behind ahead of where we started, rotating 180 degrees, now
                // facing forward
                new Pose2d(1, 0.615, new Rotation2d(Math.toRadians(179))),
                config);

        config.setReversed(true);
        // An example trajectory to follow. All units in meters.
        Trajectory depositTrajectory = TrajectoryGenerator.generateTrajectory(
                // Set the origin at (5,0) facing the +X direction
                // Robot starts facing the poles
                new Pose2d(5.3, 0.6, new Rotation2d(Math.toRadians(0))),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(
                        new Translation2d(4, 0.75) // Just kind of a test thing to see if another waypooint fixes auto

                ),
                // End 5 meters behind ahead of where we started, rotating 180 degrees, now
                // facing forward
                new Pose2d(1, 0, new Rotation2d(Math.toRadians(179))),
                config);

        var thetaController = new ProfiledPIDController(
                AutoConstants.k_pThetaController * 2, 0, 0, AutoConstants.k_thetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand goOutToCollectCommand = new SwerveControllerCommand(
                goOutToCollectTrajectory,
                s_Swerve::getPose,
                SwerveConstants.k_swerveKinematics,
                new PIDController(AutoConstants.k_pXController, 0, 0),
                new PIDController(AutoConstants.k_pYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        SwerveControllerCommand runBackToMoveBlueLineCommand = new SwerveControllerCommand(
                depositTrajectory,
                s_Swerve::getPose,
                SwerveConstants.k_swerveKinematics,
                new PIDController(AutoConstants.k_pXController, 0, 0),
                new PIDController(AutoConstants.k_pYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        addCommands(
                new InstantCommand(() -> s_Swerve.setGyroRobotFacingReverse()),
                // collect pre load command here

                // score on low high here
                new InstantCommand(() -> m_Timer.start()),
                // m_collectPreLoadCommand,
                //m_highPoleApproach,
                // m_depositSequenceCommandGroup,
                // m_intakeOff,

                // go to collect block
                new InstantCommand(() -> s_Swerve.resetOdometry(goOutToCollectTrajectory.getInitialPose()))
                // ,goOutToCollectCommand
                // ,new InstantCommand(() -> stopDT(s_Swerve, m_Timer))
                // ,m_groundCollection
                ,new AutoGroundCollectionCommandGroup(m_Lift, m_extension, s_Swerve, m_collector)

                // come to blue line for pose estimate w/ april tag
                , new InstantCommand(() -> s_Swerve.resetOdometry(depositTrajectory.getInitialPose()))
                // ,runBackToMoveBlueLineCommand

        // ,m_driveWithLimelightRIGHT

        // score!

        // new InstantCommand(() -> autoBalance(s_Swerve, m_Timer))

        );
    }

    private void stopDT(Swerve s_Swerve, Timer m_Timer) {
        TeleopSwerve m_teleopSwerve = new TeleopSwerve(
                s_Swerve,
                () -> 0,
                () -> 0,
                () -> 0,
                () -> false);

        m_teleopSwerve.execute();
    }
}