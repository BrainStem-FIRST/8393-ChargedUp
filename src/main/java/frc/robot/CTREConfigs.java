package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import frc.robot.subsystems.Swerve.SwerveConstants;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;

    public CTREConfigs(){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();

        /* Swerve Angle Motor Configurations */
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            SwerveConstants.k_angleEnableCurrentLimit, 
            SwerveConstants.k_angleContinuousCurrentLimit, 
            SwerveConstants.k_anglePeakCurrentLimit, 
            SwerveConstants.k_anglePeakCurrentDuration);

        swerveAngleFXConfig.slot0.kP = SwerveConstants.k_angleKP;
        swerveAngleFXConfig.slot0.kI = SwerveConstants.k_angleKI;
        swerveAngleFXConfig.slot0.kD = SwerveConstants.k_angleKD;
        swerveAngleFXConfig.slot0.kF = SwerveConstants.k_angleKF;
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;

        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            SwerveConstants.k_driveEnableCurrentLimit, 
            SwerveConstants.k_driveContinuousCurrentLimit, 
            SwerveConstants.k_drivePeakCurrentLimit, 
            SwerveConstants.k_drivePeakCurrentDuration);

        swerveDriveFXConfig.slot0.kP = SwerveConstants.k_driveKP;
        swerveDriveFXConfig.slot0.kI = SwerveConstants.k_driveKI;
        swerveDriveFXConfig.slot0.kD = SwerveConstants.k_driveKD;
        swerveDriveFXConfig.slot0.kF = SwerveConstants.k_driveKF;        
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.openloopRamp = SwerveConstants.k_openLoopRamp;
        swerveDriveFXConfig.closedloopRamp = SwerveConstants.k_closedLoopRamp;
        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = SwerveConstants.k_canCoderInvert;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}