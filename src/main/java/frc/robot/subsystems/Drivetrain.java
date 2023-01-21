package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.SwerveModule;
import frc.robot.utilities.SwerveModuleConstants;

public class Drivetrain extends SubsystemBase {
    public static final class DrivetrainConstants {
        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21.73); // FIXME
        public static final double wheelBase = Units.inchesToMeters(21.73); // FIXME
        public static final double wheelDiameter = Units.inchesToMeters(3.94);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = 6.12; // CORRECT!
        public static final double angleGearRatio = 150 / 7; // CORRECT!

        public static final SwerveModuleConstants frontLeftModuleConstants = new SwerveModuleConstants("Front Left",
                1, 5, false, false, 9,
                Rotation2d.fromDegrees(37.5), false); 

        public static final SwerveModuleConstants frontRightModuleConstants = new SwerveModuleConstants("Front Right",
                7, 4, false, false, 10,
                Rotation2d.fromDegrees(10.45), false); 

        public static final SwerveModuleConstants backLeftModuleConstants = new SwerveModuleConstants("Back Left",
                6, 3, false, false, 11,
                Rotation2d.fromDegrees(38.75), false); 

        public static final SwerveModuleConstants backRightModuleConstants = new SwerveModuleConstants("Back Right",
                7, 4, false, false, 10,
                Rotation2d.fromDegrees(58.88), false); 

        public static final Translation2d frontLeftPosition = new Translation2d(wheelBase / 2.0, trackWidth / 2.0);
        public static final Translation2d frontRightPosition = new Translation2d(wheelBase / 2.0, -trackWidth / 2.0);
        public static final Translation2d backLeftPosition = new Translation2d(-wheelBase / 2.0, trackWidth / 2.0);
        public static final Translation2d backRightPosition = new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0);

        public static final int gyroID = 0;
        public static final boolean gyroReversed = false; // Always ensure Gyro is CCW+ CW-

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                frontLeftPosition, frontRightPosition, backLeftPosition, backRightPosition);

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* Angle Motor PID Values */
        public static final double steerProportional = 0.6;
        public static final double steerIntegral = 0.0;
        public static final double steerDerivative = 12.0;
        public static final double steerFeedForward = 0.0;

        /* Drive Motor PID Values */
        public static final double driveProportional = 0.10;
        public static final double driveIntegral = 0.0;
        public static final double driveDerivative = 0.0;
        public static final double driveFeedForward = 0.0;

        /* Drive Motor Characterization Values */
        public static final double driveKS = (0.667 / 12); // divide by 12 to convert from volts to percent output for                                   // CTRE
        public static final double driveKV = (2.44 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        public static final double maxSpeed = 4.5; // meters per second
        public static final double maxAngularVelocity = 11.5;

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

    }

    SwerveDriveKinematics driveKinematics;
    SwerveModuleState[] swerveModuleStates;

    // swerve modules
    SwerveModule frontLeftModule;
    SwerveModule frontRightModule;
    SwerveModule backLeftModule;
    SwerveModule backRightModule;

    SwerveModule[] swerveModules;

    SwerveDriveOdometry swerveOdometry;

    Pigeon2 gyro;

    public Drivetrain() {
        gyro = new Pigeon2(DrivetrainConstants.gyroID);
        gyro.configFactoryDefault();

        frontLeftModule = new SwerveModule(DrivetrainConstants.frontLeftModuleConstants);
        frontRightModule = new SwerveModule(DrivetrainConstants.frontRightModuleConstants);
        backLeftModule = new SwerveModule(DrivetrainConstants.backLeftModuleConstants);
        backRightModule = new SwerveModule(DrivetrainConstants.backRightModuleConstants);

        swerveModules = new SwerveModule[] { frontLeftModule, frontRightModule, backLeftModule, backRightModule };

        Timer.delay(1.0);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(DrivetrainConstants.swerveKinematics, getYaw(), getModulePositions());
     

    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {

        gyro = new Pigeon2(DrivetrainConstants.gyroID);
        gyro.configFactoryDefault();
        zeroGyro();

        SwerveModuleState[] swerveModuleStates = DrivetrainConstants.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getYaw())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DrivetrainConstants.maxSpeed);

        for (SwerveModule module : swerveModules) {
            module.setDesiredState(swerveModuleStates[module.moduleNumber], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DrivetrainConstants.maxSpeed);

        for (SwerveModule module : swerveModules) {
            module.setDesiredState(desiredStates[module.moduleNumber], false);
        }
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule module : swerveModules) {
            states[module.moduleNumber] = module.getState();
        }
        return states;
    }

    public void zeroGyro() {
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        double[] ypr = new double[3];
        gyro.getYawPitchRoll(ypr);
        return (DrivetrainConstants.gyroReversed) ? Rotation2d.fromDegrees(360 - ypr[0])
                : Rotation2d.fromDegrees(ypr[0]);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule module : swerveModules){
            states[module.moduleNumber] = module.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule module : swerveModules){
            positions[module.moduleNumber] = module.getPosition();
        }
        return positions;
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : swerveModules){
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getYaw(), new SwerveModulePosition[] { frontLeftModule.getPosition(),
                frontRightModule.getPosition(), backLeftModule.getPosition(), backRightModule.getPosition() });

        for (SwerveModule module : swerveModules) {
            SmartDashboard.putNumber("Module " + module.moduleName + " Cancoder", module.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Module " + module.moduleName + " Integrated",
                    module.getState().angle.getDegrees());
            SmartDashboard.putNumber("Module " + module.moduleName + " Velocity",
                    module.getState().speedMetersPerSecond);
            SmartDashboard.putNumber("Module " + module.moduleName + " Position",
                    module.getPosition().distanceMeters);
        }
    }

    @Override
    public void simulationPeriodic() {

    }
}
