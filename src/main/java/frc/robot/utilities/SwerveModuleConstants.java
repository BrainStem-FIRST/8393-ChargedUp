package frc.robot.utilities;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
    public int driveMotorID;
    public int steerMotorID;
    public int absoluteEncoderID;

    public boolean driveMotorReversed;
    public boolean steerMotorReversed;

    public boolean absoluteEncoderReversed;
    public Rotation2d absoluteEncoderOffset;
    public String moduleName;

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * 
     * @param driveMotorID
     * @param angleMotorID
     * @param canCoderID
     * @param angleOffset
     */
    public SwerveModuleConstants(String moduleName, int driveMotorID, int steerMotorID, boolean driveMotorReversed,
            boolean steerMotorReversed, int absoluteEncoderID, Rotation2d absoluteEncoderOffset,
            boolean absoluteEncoderReversed) {
        this.moduleName = moduleName;
        this.driveMotorID = driveMotorID;
        this.steerMotorID = steerMotorID;
        this.driveMotorReversed = driveMotorReversed;
        this.steerMotorReversed = steerMotorReversed;
        this.absoluteEncoderID = absoluteEncoderID;
        this.absoluteEncoderOffset = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
    }
}
