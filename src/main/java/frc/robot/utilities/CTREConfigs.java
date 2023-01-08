package frc.robot.utilities;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import frc.robot.subsystems.Drivetrain.DrivetrainConstants;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;
    SlotConfiguration slotConfiguration;

    public CTREConfigs(boolean absoluteEncoderReversed){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();

        /* Swerve Angle Motor Configurations */
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            DrivetrainConstants.angleEnableCurrentLimit, 
            DrivetrainConstants.angleContinuousCurrentLimit, 
            DrivetrainConstants.anglePeakCurrentLimit, 
            DrivetrainConstants.anglePeakCurrentDuration);

        swerveAngleFXConfig.slot0.kP = DrivetrainConstants.steerProportional;
        swerveAngleFXConfig.slot0.kI = DrivetrainConstants.steerIntegral;
        swerveAngleFXConfig.slot0.kD = DrivetrainConstants.steerDerivative;
        swerveAngleFXConfig.slot0.kF = DrivetrainConstants.steerFeedForward;
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;
        swerveAngleFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;


        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            DrivetrainConstants.driveEnableCurrentLimit, 
            DrivetrainConstants.driveContinuousCurrentLimit, 
            DrivetrainConstants.drivePeakCurrentLimit, 
            DrivetrainConstants.drivePeakCurrentDuration);

        swerveDriveFXConfig.slot0.kP = DrivetrainConstants.driveProportional;
        swerveDriveFXConfig.slot0.kI = DrivetrainConstants.driveIntegral;
        swerveDriveFXConfig.slot0.kD = DrivetrainConstants.driveDerivative;
        swerveDriveFXConfig.slot0.kF = DrivetrainConstants.driveFeedForward;        
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        swerveDriveFXConfig.openloopRamp = DrivetrainConstants.openLoopRamp;
        swerveDriveFXConfig.closedloopRamp = DrivetrainConstants.closedLoopRamp;

        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = absoluteEncoderReversed;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;

        

    }

}
