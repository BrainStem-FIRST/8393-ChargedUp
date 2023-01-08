package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.SwerveModule;
import frc.robot.utilities.SwerveModuleConstants;

public class Drivetrain extends SubsystemBase {
    public static final class DrivetrainConstants {

        public static final SwerveModuleConstants frontLeftModuleConstants = new SwerveModuleConstants(1,
                2, false, false, 1,
                37.5, false); // FIXME

        public static final SwerveModuleConstants frontRightModuleConstants = new SwerveModuleConstants(3,
                4, false, false, 2,
                10.45, false); // FIXME

        public static final SwerveModuleConstants backLeftModuleConstants = new SwerveModuleConstants(5,
                6, false, false, 3,
                38.75, false); // FIXME

        public static final SwerveModuleConstants backRightModuleConstants = new SwerveModuleConstants(7,
                8, false, false, 4,
                58.88, false); // FIXME


        public static final int gyroID = 1;
        public static final boolean gyroReversed = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21.73); //FIXME
        public static final double wheelBase = Units.inchesToMeters(21.73); //FIXME
        public static final double wheelDiameter = Units.inchesToMeters(3.94); 
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = 6.12; //CORRECT!
        public static final double angleGearRatio = 150/7; //CORRECT!

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

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
        public static final double driveKS = (0.667 / 12); //divide by 12 to convert from volts to percent output for CTRE
        public static final double driveKV = (2.44 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        public static final double maxSpeed = 4.5; //meters per second
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

    ArrayList<SwerveModule> swerveModules;

    SwerveDriveOdometry swerveOdometry;

    Pigeon2 gyro;

    public Drivetrain() {
        swerveModuleStates = driveKinematics.toSwerveModuleStates(null);

        frontLeftModule = new SwerveModule(DrivetrainConstants.frontLeftModuleConstants);
        frontRightModule = new SwerveModule(DrivetrainConstants.frontRightModuleConstants);
        backLeftModule = new SwerveModule(DrivetrainConstants.backLeftModuleConstants);
        backRightModule = new SwerveModule(DrivetrainConstants.backRightModuleConstants);

        swerveModules = new ArrayList<>(
                Arrays.asList(frontLeftModule, frontRightModule, backLeftModule, backRightModule));

        gyro = new Pigeon2(DrivetrainConstants.gyroID);      
    }

    public CommandBase exampleMethodCommand() {
        return runOnce(
                () -> {

                });
    }

    public boolean exampleCondition() {
        return false;
    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {

    }
}
