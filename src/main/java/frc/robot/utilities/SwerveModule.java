package frc.robot.utilities;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.Drivetrain.DrivetrainConstants;

public class SwerveModule {
    private TalonFX driveMotor;
    private TalonFX steerMotor;
    private int driveMotorID;
    private int steerMotorID;
    private int absoluteEncoderID;

    private boolean driveMotorReversed;
    private boolean steerMotorReversed;

    private CANCoder absoluteEncoder;

    private boolean absoluteEncoderReversed;
    private double absoluteEncoderOffset;

    private CTREConfigs swerveModuleConfig;

    private double lastAngle;

    public String moduleName;

    public int moduleNumber;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DrivetrainConstants.driveKS,
            DrivetrainConstants.driveKV, DrivetrainConstants.driveKA);

    public SwerveModule(SwerveModuleConstants swerveModuleConstants) {
        this.moduleName = swerveModuleConstants.moduleName;
        this.driveMotorID = swerveModuleConstants.driveMotorID;
        this.steerMotorID = swerveModuleConstants.steerMotorID;
        this.driveMotorReversed = swerveModuleConstants.driveMotorReversed;
        this.steerMotorReversed = swerveModuleConstants.steerMotorReversed;
        this.absoluteEncoderID = swerveModuleConstants.absoluteEncoderID;
        this.absoluteEncoderOffset = swerveModuleConstants.absoluteEncoderOffset;
        this.absoluteEncoderReversed = swerveModuleConstants.absoluteEncoderReversed;
        this.swerveModuleConfig = new CTREConfigs(swerveModuleConstants.absoluteEncoderReversed);
        swerveModuleConfig.swerveCanCoderConfig.sensorDirection = this.absoluteEncoderReversed;

        driveMotor = new TalonFX(driveMotorID);
        steerMotor = new TalonFX(steerMotorID);
        absoluteEncoder = new CANCoder(absoluteEncoderID);

        configDriveMotor();
        configAngleMotor();
        configAbsoluteEncoder();

        lastAngle = getState().angle.getDegrees();

        switch (moduleName) {
            case "Front Left": {
                moduleNumber = 0;
                break;
            }
            case "Front Right": {
                moduleNumber = 1;
                break;
            }
            case "Back Left": {
                moduleNumber = 2;
                break;
            }
            case "Back Right": {
                moduleNumber = 3;
                break;
            }
            default: {
                moduleNumber = -1;
                break;
            }
        }
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); // Custom optimize command, since
                                                                                 // default WPILib optimize assumes
                                                                                 // continuous controller which CTRE is
                                                                                 // not

        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / DrivetrainConstants.maxSpeed;
            driveMotor.set(ControlMode.PercentOutput, percentOutput);
        } else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond,
                    DrivetrainConstants.wheelCircumference, DrivetrainConstants.driveGearRatio);
            driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward,
                    feedforward.calculate(desiredState.speedMetersPerSecond));
        }

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (DrivetrainConstants.maxSpeed * 0.01))
                ? lastAngle
                : desiredState.angle.getDegrees(); // Prevent rotating module if speed is less then 1%. Prevents
                                                   // Jittering.
        steerMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle, DrivetrainConstants.angleGearRatio));
        lastAngle = angle;
    }

    private void resetToAbsolute() {
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - absoluteEncoderOffset,
                DrivetrainConstants.angleGearRatio);
        steerMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configAbsoluteEncoder() {
        absoluteEncoder.configFactoryDefault();
        absoluteEncoder.configAllSettings(swerveModuleConfig.swerveCanCoderConfig);
    }

    private void configAngleMotor() {
        steerMotor.configFactoryDefault();
        steerMotor.configAllSettings(swerveModuleConfig.swerveAngleFXConfig);
        steerMotor.setInverted(steerMotorReversed);
        steerMotor.setNeutralMode(DrivetrainConstants.angleNeutralMode);
        resetToAbsolute();
    }

    private void configDriveMotor() {
        driveMotor.configFactoryDefault();
        driveMotor.configAllSettings(swerveModuleConfig.swerveDriveFXConfig);
        driveMotor.setInverted(driveMotorReversed);
        driveMotor.setNeutralMode(DrivetrainConstants.driveNeutralMode);
        driveMotor.setSelectedSensorPosition(0);
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(absoluteEncoder.getAbsolutePosition());
    }

    public SwerveModuleState getState() {
        double velocity = Conversions.falconToMPS(driveMotor.getSelectedSensorVelocity(),
                DrivetrainConstants.wheelCircumference, DrivetrainConstants.driveGearRatio);
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(steerMotor.getSelectedSensorPosition(),
                DrivetrainConstants.angleGearRatio));
        return new SwerveModuleState(velocity, angle);
    }

    public SwerveModulePosition getPosition() {
        double distance = driveMotor.getSelectedSensorPosition();
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(steerMotor.getSelectedSensorPosition(),
                DrivetrainConstants.angleGearRatio));
        return new SwerveModulePosition(velocity, angle);
    }

}