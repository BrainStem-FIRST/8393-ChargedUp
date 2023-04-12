package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autoCommands.DriveForwardUntilLimelight;
import frc.robot.commands.MonkDrive;
import frc.robot.commands.WaitCommand;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Collector.IntakeState;
import frc.robot.subsystems.Lift.LiftPosition;

public class AutoGroundCollectionCommandGroup extends SequentialCommandGroup{
    private Lift m_lift;
    private Collector m_collector;
    private Swerve m_swerve;
    private Extension m_extension;
    public AutoGroundCollectionCommandGroup(Lift p_lift, Extension p_extension, Swerve p_swerve, Collector p_collector) {
        this.m_lift = p_lift;
        this.m_collector = p_collector;
        this.m_swerve = p_swerve;
        this.m_extension = p_extension;

        addRequirements(m_lift, m_collector, m_swerve, m_extension);
        addCommands(
            new MonkDrive(m_swerve, true),
            new InstantCommand(() -> m_lift.m_state = LiftPosition.CARRY),
            new WaitCommand(.1),
            new InstantCommand(() -> m_collector.m_intakeState = IntakeState.IN),
            new GroundCollectionCommandGroup(m_extension, m_lift, m_collector),
            new DriveForwardUntilLimelight(m_swerve)
            );

    }
    
}
