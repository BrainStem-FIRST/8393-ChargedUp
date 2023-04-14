package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import frc.robot.SwerveModule;
import frc.robot.utilities.BrainSTEMSubsystem;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.LimelightHelpers;

public class Swerve extends SubsystemBase implements BrainSTEMSubsystem {
    public static final class SwerveConstants {
        public static final int k_pigeonID = 13;
        public static final boolean k_invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants k_chosenModule = // TODO: This must be tuned to specific robot
                COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L3);

        /* Drivetrain Constants */
        public static final double k_trackWidth = Units.inchesToMeters(21.73); // TODO: This must be tuned to specific
                                                                               // robot
        public static final double k_wheelBase = Units.inchesToMeters(21.73); // TODO: This must be tuned to specific
                                                                              // robot
        public static final double k_wheelCircumference = k_chosenModule.wheelCircumference;

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics k_swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(k_wheelBase / 2.0, k_trackWidth / 2.0),
                new Translation2d(k_wheelBase / 2.0, -k_trackWidth / 2.0),
                new Translation2d(-k_wheelBase / 2.0, k_trackWidth / 2.0),
                new Translation2d(-k_wheelBase / 2.0, -k_trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double k_driveGearRatio = k_chosenModule.driveGearRatio;
        public static final double k_angleGearRatio = k_chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean k_angleMotorInvert = k_chosenModule.angleMotorInvert;
        public static final boolean k_driveMotorInvert = k_chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean k_canCoderInvert = k_chosenModule.canCoderInvert;

        public static final double k_drivetrainLimit = 1;

        /* Swerve Current Limiting */
        public static final int k_angleContinuousCurrentLimit = 25;
        public static final int k_anglePeakCurrentLimit = 40;
        public static final double k_anglePeakCurrentDuration = 0.1;
        public static final boolean k_angleEnableCurrentLimit = true;

        public static final int k_driveContinuousCurrentLimit = 35;
        public static final int k_drivePeakCurrentLimit = 60;
        public static final double k_drivePeakCurrentDuration = 0.1;
        public static final boolean k_driveEnableCurrentLimit = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double k_openLoopRamp = 0.25;
        public static final double k_closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double k_angleKP = k_chosenModule.angleKP;
        public static final double k_angleKI = k_chosenModule.angleKI;
        public static final double k_angleKD = k_chosenModule.angleKD;
        public static final double k_angleKF = k_chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double k_driveKP = 0.05; // TODO: This must be tuned to specific robot
        public static final double k_driveKI = 0.0;
        public static final double k_driveKD = 0.0;
        public static final double k_driveKF = 0.0;

        /*
         * Drive Motor Characterization Values
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE
         */
        public static final double k_driveKS = (0.32 / 12); // TODO: This must be tuned to specific robot
        public static final double k_driveKV = (1.51 / 12);
        public static final double k_driveKA = (0.27 / 12);

        public static final PIDConstants translationConstants = new PIDConstants(4.0, 0, 0);
        public static final PIDConstants thetaConstants = new PIDConstants(4.0, 0, 0);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double k_maxSpeed = 4.5; // TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double k_maxAngularVelocity = 10.0; // TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralMode k_angleNeutralMode = NeutralMode.Brake;
        public static final NeutralMode k_driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0Constants { // TODO: This must be tuned to specific robot
            public static final int k_driveMotorID = 1;
            public static final int k_angleMotorID = 5;
            public static final int k_canCoderID = 9;
            public static final Rotation2d k_angleOffset = Rotation2d.fromDegrees(228.16);
            public static final SwerveModuleConstants k_constants = new SwerveModuleConstants(k_driveMotorID,
                    k_angleMotorID, k_canCoderID, k_angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1Constants { // TODO: This must be tuned to specific robot
            public static final int k_driveMotorID = 7;
            public static final int k_angleMotorID = 4;
            public static final int k_canCoderID = 10;
            public static final Rotation2d k_angleOffset = Rotation2d.fromDegrees(344.65);
            public static final SwerveModuleConstants k_constants = new SwerveModuleConstants(k_driveMotorID,
                    k_angleMotorID, k_canCoderID, k_angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2Constants { // TODO: This must be tuned to specific robot
            public static final int k_driveMotorID = 6;
            public static final int k_angleMotorID = 3;
            public static final int k_canCoderID = 11;
            public static final Rotation2d k_angleOffset = Rotation2d.fromDegrees(350.22);
            public static final SwerveModuleConstants k_constants = new SwerveModuleConstants(k_driveMotorID,
                    k_angleMotorID, k_canCoderID, k_angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3Constants { // TODO: This must be tuned to specific robot
            public static final int k_driveMotorID = 2;
            public static final int k_angleMotorID = 8;
            public static final int k_canCoderID = 12;
            public static final Rotation2d k_angleOffset = Rotation2d.fromDegrees(334.6);
            public static final SwerveModuleConstants k_constants = new SwerveModuleConstants(k_driveMotorID,
                    k_angleMotorID, k_canCoderID, k_angleOffset);
        }
    }

    public SwerveDrivePoseEstimator swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    public boolean m_enableSwervePeriodic = false;

    public NeutralMode m_adjustableDriveNeutralMode = SwerveConstants.k_driveNeutralMode;
    public NeutralMode m_adjustableAngleNeutralMode = SwerveConstants.k_angleNeutralMode;

    public SwerveModuleState[] swerveModuleStates;

    public boolean isOpenLoop;
    public Field2d field2d ;
    public Swerve() {
        
        gyro = new Pigeon2(SwerveConstants.k_pigeonID, "CANivore_dt");
        gyro.configFactoryDefault();
        zeroGyro();

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, SwerveConstants.Mod0Constants.k_constants),
                new SwerveModule(1, SwerveConstants.Mod1Constants.k_constants),
                new SwerveModule(2, SwerveConstants.Mod2Constants.k_constants),
                new SwerveModule(3, SwerveConstants.Mod3Constants.k_constants)
        };

        field2d = new Field2d();
        SmartDashboard.putData("Field", field2d);

        /*
         * By pausing init for a second before setting module offsets, we avoid a bug
         * with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDrivePoseEstimator(SwerveConstants.k_swerveKinematics, getYaw(), getModulePositions(), new Pose2d());
    }

    public SwerveAutoBuilder getAutoBuilder(HashMap<String, Command> eventMap) {
        return new SwerveAutoBuilder(this::getPose, this::resetOdometry, SwerveConstants.k_swerveKinematics,
                SwerveConstants.translationConstants, SwerveConstants.thetaConstants, this::setModuleStates, eventMap,
                true, this);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {

        this.isOpenLoop = isOpenLoop;
        swerveModuleStates = SwerveConstants.k_swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getYaw())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.k_maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    @Override
    public void initialize() {

        m_adjustableDriveNeutralMode = SwerveConstants.k_driveNeutralMode;
        m_adjustableAngleNeutralMode = SwerveConstants.k_angleNeutralMode;
        enablePeriodic();
    }

    @Override
    public void enablePeriodic() {
        m_enableSwervePeriodic = true;
    }

    @Override
    public void disablePeriodic() {
        m_enableSwervePeriodic = false;
    }

    // public Command alignToTag() {
    // //Pose3d tagPose =
    // }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.k_maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public Pose2d getPose() {
        return swerveOdometry.getEstimatedPosition();
    }

    public void updatePoseWithVision() {
        swerveOdometry.addVisionMeasurement(
                LimelightHelpers.getBotPose2d("limelight-a"),
                LimelightHelpers.getLatency_Capture("limelight-a"));
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro() {
        gyro.setYaw(0);
    }

    public void setGyroRobotFacingReverse() {
        gyro.setYaw(180);
    }

    public Rotation2d getYaw() {
        return (SwerveConstants.k_invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw())
                : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public double getTilt() {
        return gyro.getPitch();
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    public CommandBase resetModuleBase() {
        return runOnce(this::resetModulesToAbsolute);
    }

    @Override
    public void periodic() {
        if (m_enableSwervePeriodic) { // m_enableSwervePeriodic
            swerveOdometry.update(getYaw(), getModulePositions());
            updatePoseWithVision();
            for (SwerveModule mod : mSwerveMods) {
                mod.setModuleNeutralMode(m_adjustableDriveNeutralMode, m_adjustableAngleNeutralMode);

            }
            field2d.setRobotPose(getPose());

            for (SwerveModule mod : mSwerveMods) {
                SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
                SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated",
                        mod.getPosition().angle.getDegrees());
                SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
            }
            SmartDashboard.putNumber("Robot Tilt", getTilt());
            SmartDashboard.putNumber("Robot Heading", getYaw().getDegrees());
        }
    }
}