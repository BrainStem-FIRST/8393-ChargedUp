package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.SwerveModule;
import frc.robot.utilities.SwerveModuleConstants;

public class Drivetrain extends SubsystemBase {
    private static final class DrivetrainConstants {
        private static final Translation2d frontLeftModuleLocation = new Translation2d(1, 1); // FIXME
        private static final Translation2d frontRightModuleLocation = new Translation2d(1, -1); // FIXME
        private static final Translation2d backLeftModuleLocation = new Translation2d(-1, 1); // FIXME
        private static final Translation2d backRightModuleLocation = new Translation2d(-1, -1); // FIXME

        private static final SwerveModuleConstants frontLeftModuleConstants = new SwerveModuleConstants(1,
                2, false, false, 0,
                0, false); //FIXME

        private static final SwerveModuleConstants frontRightModuleConstants = new SwerveModuleConstants(1,
                2, false, false, 0,
                0, false); //FIXME

        private static final SwerveModuleConstants backLeftModuleConstants = new SwerveModuleConstants(1,
                2, false, false, 0,
                0, false); //FIXME

        private static final SwerveModuleConstants backRightModuleConstants = new SwerveModuleConstants(1,
                2, false, false, 0,
                0, false); //FIXME

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

        frontLeftModule = new SwerveModule(DrivetrainConstants.frontLeftModuleConstants);
        frontRightModule = new SwerveModule(DrivetrainConstants.frontRightModuleConstants);
        backLeftModule = new SwerveModule(DrivetrainConstants.backLeftModuleConstants);
        backRightModule = new SwerveModule(DrivetrainConstants.backRightModuleConstants);
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
