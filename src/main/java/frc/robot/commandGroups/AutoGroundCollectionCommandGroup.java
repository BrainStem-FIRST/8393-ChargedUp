package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autoCommands.DriveForwardUntilLimelight;
import frc.robot.commands.MonkDrive;
import frc.robot.commands.WaitCommand;
import frc.robot.commands.collectorCommands.CollectorOpenCommand;
import frc.robot.commands.liftCommands.LiftCarryCommand;
import frc.robot.commands.liftCommands.LiftGroundCommand;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Collector.CollectorConstants;
import frc.robot.subsystems.Collector.IntakeState;
import frc.robot.subsystems.Extension.TelescopePosition;
import frc.robot.subsystems.Lift.LiftConstants;
import frc.robot.subsystems.Lift.LiftPosition;

public class AutoGroundCollectionCommandGroup extends SequentialCommandGroup {
    private Lift m_lift;
    private Collector m_collector;
    private Extension m_extension;

    public AutoGroundCollectionCommandGroup(Extension p_extension, Lift p_lift, Collector p_collector) {
        this.m_lift = p_lift;
        this.m_collector = p_collector;
        this.m_extension = p_extension;

        addRequirements(m_lift, m_collector, m_extension);
        addCommands(
                new InstantCommand(
                        () -> m_collector.m_adjustableWheelMotorPower = CollectorConstants.k_wheelMotorSpeed / 2),
                new InstantCommand(() -> m_collector.m_intakeState = IntakeState.IN),
                new LiftCarryCommand(m_lift),
                new WaitCommand(1),
                new InstantCommand(() -> m_extension.m_telescopeState = TelescopePosition.AUTO_CUBE_COLLECT),
                new InstantCommand(() -> m_lift.m_adjustableLiftSpeed = LiftConstants.k_MaxPower / 2),
                new WaitCommand(0.25),
                new LiftGroundCommand(m_lift),
                new InstantCommand(() -> m_lift.m_adjustableLiftSpeed = LiftConstants.k_MaxPower),
                new CollectorOpenCommand(m_collector));

    }

}
