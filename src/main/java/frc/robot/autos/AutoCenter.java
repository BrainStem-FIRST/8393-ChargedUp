package frc.robot.autos;

import frc.robot.autoCommands.LiftCollectPreLoad;
import frc.robot.autoCommands.autoCommandGroups.collectPreLoadCommand;
import frc.robot.commandGroups.DepositSequenceCommandGroup;
import frc.robot.commandGroups.HighPoleApproachCommandGroup;
import frc.robot.commandGroups.LowPoleApproachCommandGroup;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.WaitCommand;
import frc.robot.commands.collectorCommands.CollectorCloseCommand;
import frc.robot.commands.collectorCommands.IntakeInCommand;
import frc.robot.commands.collectorCommands.IntakeOffCommand;
import frc.robot.commands.extensionCommands.ExtensionCommand;
import frc.robot.commands.liftCommands.LiftGroundCommand;
import frc.robot.commands.liftCommands.RaiseToUnlockRatchetCommand;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Collector.CollectorConstants;
import frc.robot.subsystems.Collector.CollectorState;
import frc.robot.subsystems.Extension.TelescopePosition;
import frc.robot.subsystems.Lift.LiftConstants;
import frc.robot.subsystems.Lift.LiftPosition;
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
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.math.trajectory.constraint.RectangularRegionConstraint;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj.Timer;

public class AutoCenter extends SequentialCommandGroup {

        public static final class AutoConstants { // TODO: The below constants are used in the example auto, and must be
                                                  // tuned to specific robot
                public static final double k_maxSpeedMetersPerSecond = 1.8;
                public static final double k_maxAccelerationMetersPerSecondSquared = 0.9;
                public static final double k_maxAngularSpeedRadiansPerSecond = Math.PI / 10;
                public static final double k_maxAngularSpeedRadiansPerSecondSquared = Math.PI / 10;

                public static final double k_pXController = 1;
                public static final double k_pYController = 0;
                public static final double k_pThetaController = 0;

                /* Constraint for the motion profilied robot angle controller */
                public static final TrapezoidProfile.Constraints k_thetaControllerConstraints = new TrapezoidProfile.Constraints(
                                k_maxAngularSpeedRadiansPerSecond, k_maxAngularSpeedRadiansPerSecondSquared);
        }

        private static final class BalanceConstants {
                private static final double k_P = 0.8;
                private static final double k_I = 0.000;
                private static final double k_D = 0.0;
        }

        public AutoCenter(Swerve s_Swerve, Lift m_lift, Collector m_collector, Extension m_extension) {
                collectPreLoadCommand m_collectPreLoadCommand = new collectPreLoadCommand(m_lift, m_collector);
                ExtensionCommand m_extensionCarry = new ExtensionCommand(m_extension, TelescopePosition.COLLECTION);
                LowPoleApproachCommandGroup m_lowPoleApproach = new LowPoleApproachCommandGroup(m_extension, m_lift,
                                m_collector);
                DepositSequenceCommandGroup m_depositSequenceCommandGroup = new DepositSequenceCommandGroup(m_lift,
                                m_extension,
                                m_collector);
                IntakeOffCommand m_intakeOff = new IntakeOffCommand(m_collector);
                IntakeInCommand m_intakeIn = new IntakeInCommand(m_collector);
                HighPoleApproachCommandGroup m_highPoleApproach = new HighPoleApproachCommandGroup(m_extension, m_lift,
                                m_collector);
                Timer m_Timer = new Timer();

                RectangularRegionConstraint drivetrainRestraint = new RectangularRegionConstraint(
                                new Translation2d(3.814, 1.462), new Translation2d(12.75, 3.945),
                                new MaxVelocityConstraint(AutoConstants.k_maxSpeedMetersPerSecond / 5));

                TrajectoryConfig config = new TrajectoryConfig(
                                AutoConstants.k_maxSpeedMetersPerSecond,
                                AutoConstants.k_maxAccelerationMetersPerSecondSquared)
                                .setKinematics(SwerveConstants.k_swerveKinematics).addConstraint(drivetrainRestraint);

                TrajectoryConfig runBackOntoChargeStationConfig = new TrajectoryConfig(
                                AutoConstants.k_maxSpeedMetersPerSecond,
                                AutoConstants.k_maxAccelerationMetersPerSecondSquared)
                                .setKinematics(SwerveConstants.k_swerveKinematics);

                config.setReversed(true);
                // An example trajectory to follow. All units in meters.
                Trajectory runOverChargeStationTrajectory = TrajectoryGenerator.generateTrajectory(
                                // Set the origin at (5,0) facing the +X direction
                                // Robot starts facing the poles
                                new Pose2d(5.5, 0, new Rotation2d(Math.toRadians(0))),
                                // Pass through these two interior waypoints, making an 's' curve path
                                List.of(
                                                new Translation2d(2, 0) // Just kind of a test thing to see if another
                                                                        // waypooint fixes auto
                                ),
                                // End 5 meters behind ahead of where we started, rotating 180 degrees, now
                                // facing forward
                                new Pose2d(1, 0, new Rotation2d(Math.toRadians(0))),
                                config);

                config.setReversed(false);
                // An example trajectory to follow. All units in meters.
                Trajectory runBackOnToChargeStationTrajectory = TrajectoryGenerator.generateTrajectory(
                                // Set the origin at (5,0) facing the +X direction
                                // Robot starts facing the poles
                                new Pose2d(1, 0, new Rotation2d(Math.toRadians(0))),
                                // Pass through these two interior waypoints, making an 's' curve path
                                List.of(
                                                new Translation2d(2, 0) // Just kind of a test thing to see if another
                                                                        // waypooint fixes auto
                                ),
                                // End 5 meters behind ahead of where we started, rotating 180 degrees, now
                                // facing forward
                                new Pose2d(3.65, 0, new Rotation2d(Math.toRadians(0))), //3.6 before
                                runBackOntoChargeStationConfig);

                var thetaController = new ProfiledPIDController(
                                AutoConstants.k_pThetaController * 2, 0, 0, AutoConstants.k_thetaControllerConstraints);
                thetaController.enableContinuousInput(-Math.PI, Math.PI);

                SwerveControllerCommand runOverChargeStationCommand = new SwerveControllerCommand(
                                runOverChargeStationTrajectory,
                                s_Swerve::getPose,
                                SwerveConstants.k_swerveKinematics,
                                new PIDController(AutoConstants.k_pXController, 0, 0),
                                new PIDController(AutoConstants.k_pYController, 0, 0),
                                thetaController,
                                s_Swerve::setModuleStates,
                                s_Swerve);

                SwerveControllerCommand runBackOntoChargeStationCommand = new SwerveControllerCommand(
                                runBackOnToChargeStationTrajectory,
                                s_Swerve::getPose,
                                SwerveConstants.k_swerveKinematics,
                                new PIDController(AutoConstants.k_pXController, 0, 0),
                                new PIDController(AutoConstants.k_pYController, 0, 0),
                                thetaController,
                                s_Swerve::setModuleStates,
                                s_Swerve);

                addCommands(
                                new WaitCommand(0.1),
                                new InstantCommand(() -> m_collector.m_collectorState = CollectorState.CLOSED),
                                new InstantCommand(() -> s_Swerve.setGyroRobotFacingReverse()),
                                // collect pre load command here

                                // score on low pole command here
                                new InstantCommand(() -> m_Timer.start()),
                                // new RaiseToUnlockRatchetCommand(m_lift),
                                // new InstantCommand(() -> m_lift.m_state = LiftPosition.RATCHET),
                                
                                // new InstantCommand(() -> m_lift.m_state = LiftPosition.HIGH_POLE),

                                new InstantCommand(() -> s_Swerve
                                                .resetOdometry(runOverChargeStationTrajectory.getInitialPose())),
                                m_highPoleApproach,
                                m_depositSequenceCommandGroup,
                                m_intakeOff,
                                runOverChargeStationCommand,
                                runBackOntoChargeStationCommand,

                                new InstantCommand(() -> autoBalance(s_Swerve, m_Timer))

                );
        }

        private void autoBalance(Swerve s_Swerve, Timer m_Timer) {
                double pitch = s_Swerve.getTilt();
                PIDController m_balancePID = new PIDController(BalanceConstants.k_P, BalanceConstants.k_I,
                                BalanceConstants.k_D);
                m_balancePID.setTolerance(10);
                int counter = 0;
                boolean balancing = false;

                pitch = s_Swerve.getTilt();

                // final double initialPower = 0.4;

                // while (pitch < 10) {
                // pitch = s_Swerve.getTilt();
                // TeleopSwerve m_teleopSwerve = new TeleopSwerve(
                // s_Swerve,
                // () -> initialPower,
                // () -> 0,
                // () -> 0,
                // () -> false
                // );
                // m_teleopSwerve.execute();
                // }

                // while (pitch > 10) {
                // pitch = s_Swerve.getTilt();

                // TeleopSwerve m_teleopSwerve = new TeleopSwerve(
                // s_Swerve,
                // () -> 0.25,
                // () -> 0,
                // () -> 0,
                // () -> false
                // );

                // m_teleopSwerve.execute();

                // SmartDashboard.putNumber("Auto Pitch", pitch);
                // SmartDashboard.putBoolean("Balance Power", balancing);
                // SmartDashboard.putNumber("Counter", counter);
                // }

                // TeleopSwerve m_teleopSwerve = new TeleopSwerve(
                // s_Swerve,
                // () -> 0,
                // () -> 0,
                // () -> 0,
                // () -> false
                // );

                // m_teleopSwerve.execute();

                SmartDashboard.putNumber("Auto Pitch", pitch);
                SmartDashboard.putBoolean("Balance Power", balancing);
                SmartDashboard.putNumber("Counter", counter);

                TeleopSwerve m_teleopSwerve = new TeleopSwerve(
                                s_Swerve,
                                () -> 0,
                                () -> 0,
                                () -> 0,
                                () -> false);

                int x = 0;

                while (pitch > 5 && x < 250000) { //8
                        pitch = s_Swerve.getTilt();
                     
                        double power = MathUtil.clamp(m_balancePID.calculate(pitch, 0), -0.25, 0.25);

                        m_teleopSwerve = new TeleopSwerve(
                                        s_Swerve,
                                        () -> -power * 0.73,
                                        () -> 0,
                                        () -> 0,
                                        () -> false);

                        x++;
                        m_teleopSwerve.execute();
                        counter++;
                        SmartDashboard.putNumber("Auto Pitch", pitch);
                        SmartDashboard.putNumber("Balance Power", power);
                        SmartDashboard.putNumber("Counter", counter);
                        SmartDashboard.putNumber("Timer", m_Timer.getFPGATimestamp());
                }

                m_teleopSwerve = new TeleopSwerve(
                                s_Swerve,
                                () -> 0,
                                () -> 0,
                                () -> 0.3,
                                () -> false);
                m_teleopSwerve.execute();

        }
}