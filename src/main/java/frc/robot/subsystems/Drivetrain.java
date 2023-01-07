package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
    private static final class DrivetrainConstants {
        private static final Translation2d frontLeftModuleLocation = new Translation2d(1, 1); // FIXME
        private static final Translation2d frontRightModuleLocation = new Translation2d(1, -1); // FIXME
        private static final Translation2d backLeftModuleLocation = new Translation2d(-1, 1); // FIXME
        private static final Translation2d backRightModuleLocation = new Translation2d(-1, -1); // FIXME

        private static final int frontLeftDriveMotorID = 0; // FIXME
        private static final int frontLeftSteerMotorID = 1; // FIXME

        private static final int frontRightDriveMotorID = 2; // FIXME
        private static final int frontRightSteerMotorID = 3; // FIXME

        private static final int backLeftDriveMotorID = 4; // FIXME
        private static final int backLeftSteerMotorID = 5; // FIXME

        private static final int backRightDriveMotorID = 6; // FIXME
        private static final int backRightSteerMotorID = 7; // FIXME


        private static final boolean frontLeftDriveMotorReversed = false; // FIXME
        private static final boolean frontLeftSteerMotorReversed = false; // FIXME

        private static final boolean frontRightDriveMotorReversed = false; // FIXME
        private static final boolean frontRightSteerMotorReversed = false; // FIXME

        private static final boolean backLeftDriveMotorReversed = false; // FIXME
        private static final boolean backLeftSteerMotorReversed = false; // FIXME

        private static final boolean backRightDriveMotorReversed = false; // FIXME
        private static final boolean backRightSteerMotorReversed = false; // FIXME

        private static final int frontLeftAbsoluteEncoderID = 8; //FIXME
        private static final int frontRightAbsoluteEncoderID = 9; //FIXME
        private static final int backLeftAbsoluteEncoderID = 10; //FIXME
        private static final int backRightAbsoluteEncoderID = 11; //FIXME

    }

    SwerveDriveKinematics driveKinematics;
    SwerveModuleState[] swerveModuleStates;

    // swerve modules
    SwerveModule frontLeftModule;
    SwerveModule frontRightModule;
    SwerveModule backLeftModule;
    SwerveModule backRightModule;

    public Drivetrain() {
        driveKinematics = new SwerveDriveKinematics(DrivetrainConstants.frontLeftModuleLocation,
                DrivetrainConstants.frontRightModuleLocation, DrivetrainConstants.backLeftModuleLocation,
                DrivetrainConstants.backRightModuleLocation);
        swerveModuleStates = driveKinematics.toSwerveModuleStates(null);

        /*frontLeftModule = new SwerveModule(DrivetrainConstants.frontLeftDriveMotorID,
                DrivetrainConstants.frontLeftSteerMotorID);
        frontRightModule = new SwerveModule(DrivetrainConstants.frontRightDriveMotorID,
                DrivetrainConstants.frontRightSteerMotorID);
        backLeftModule = new SwerveModule(DrivetrainConstants.backLeftDriveMotorID,
                DrivetrainConstants.backLeftSteerMotorID);
        backRightModule = new SwerveModule(DrivetrainConstants.backRightDriveMotorID,
                DrivetrainConstants.backRightSteerMotorID);*/
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
