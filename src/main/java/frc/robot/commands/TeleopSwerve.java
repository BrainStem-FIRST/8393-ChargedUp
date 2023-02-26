package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Swerve.SwerveConstants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopSwerve extends CommandBase {
    private Swerve s_Swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup,
            DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void execute() {
            double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(),
                    RobotContainer.k_stickDeadband);
            double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), RobotContainer.k_stickDeadband);
            //double rotationVal = MathUtil.applyDeadband(Math.pow(rotationSup.getAsDouble() * 0.2, 3), RobotContainer.k_stickDeadband);
            double rotationVal = 0.5* MathUtil.applyDeadband(rotationSup.getAsDouble(), RobotContainer.k_stickDeadband);

            /* Drive */
            s_Swerve.drive(
                    new Translation2d(translationVal, strafeVal).times(SwerveConstants.k_maxSpeed),
                    rotationVal * SwerveConstants.k_maxAngularVelocity,
                    !robotCentricSup.getAsBoolean(),
                    true);
    }
}