package frc.robot.utilities;

public class SwerveModuleConstants {
    public int driveMotorID;
    public int steerMotorID;
    public int absoluteEncoderID;

    public boolean driveMotorReversed;
    public boolean steerMotorReversed;

    public boolean absoluteEncoderReversed;
    public double absoluteEncoderOffset;

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * 
     * @param driveMotorID
     * @param angleMotorID
     * @param canCoderID
     * @param angleOffset
     */
    public SwerveModuleConstants(int driveMotorID, int steerMotorID, boolean driveMotorReversed,
            boolean steerMotorReversed, int absoluteEncoderID, double absoluteEncoderOffset,
            boolean absoluteEncoderReversed) {
        this.driveMotorID = driveMotorID;
        this.steerMotorID = steerMotorID;
        this.driveMotorReversed = driveMotorReversed;
        this.steerMotorReversed = steerMotorReversed;
        this.absoluteEncoderID = absoluteEncoderID;
        this.absoluteEncoderOffset = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
    }
}
