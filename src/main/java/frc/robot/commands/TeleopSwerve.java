package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.DrivetrainConstants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopSwerve extends CommandBase {
    private static final class TeleopSwerveConstants {
        private static final double driver1Deadzone = 0.05;
    }

    private Drivetrain drivetrain;
    private DoubleSupplier translationSupplier;
    private DoubleSupplier strafeSupplier;
    private DoubleSupplier rotationSupplier;
    private BooleanSupplier robotCentricSupplier;

    public TeleopSwerve(Drivetrain drivetrain, DoubleSupplier translationSupplier,
            DoubleSupplier strafeSupplier, DoubleSupplier rotationSupplier, BooleanSupplier robotCentricSupplier) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);

        this.translationSupplier = translationSupplier;
        this.strafeSupplier = strafeSupplier;
        this.rotationSupplier = rotationSupplier;
        this.robotCentricSupplier = robotCentricSupplier;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband */
        double translationVal = MathUtil.applyDeadband(translationSupplier.getAsDouble(),
                TeleopSwerveConstants.driver1Deadzone);
        double strafeVal = MathUtil.applyDeadband(strafeSupplier.getAsDouble(), TeleopSwerveConstants.driver1Deadzone);
        double rotationVal = MathUtil.applyDeadband(rotationSupplier.getAsDouble(),
                TeleopSwerveConstants.driver1Deadzone);

        /* Drive */
        drivetrain.drive(
                new Translation2d(translationVal, strafeVal).times(DrivetrainConstants.maxSpeed),
                rotationVal * DrivetrainConstants.maxAngularVelocity,
                !robotCentricSupplier.getAsBoolean(),
                true);
    }
}
