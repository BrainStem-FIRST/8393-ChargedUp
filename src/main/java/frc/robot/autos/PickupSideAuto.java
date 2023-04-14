package frc.robot.autos;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commandGroups.AutoDepositSequenceCommandGroup;
import frc.robot.commandGroups.AutoGroundCollectionCommandGroup;
import frc.robot.commandGroups.AutoHighPoleApproachCommandGroup;
import frc.robot.commandGroups.CarryRetractedCommandGroup;
import frc.robot.commandGroups.DepositSequenceCommandGroup;
import frc.robot.commandGroups.GroundCollectionCommandGroup;
import frc.robot.commandGroups.HighPoleApproachCommandGroup;
import frc.robot.commands.WaitCommand;
import frc.robot.commands.collectorCommands.IntakeOffCommand;
import frc.robot.commands.liftCommands.RaiseToUnlockRatchetCommand;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Collector.CollectorConstants;
import frc.robot.subsystems.Collector.IntakeState;
import frc.robot.subsystems.Lift.LiftPosition;

public class PickupSideAuto extends SequentialCommandGroup {

        public static final class PickupSideAutoConstants {
                public static final PathConstraints goOutToCollectConstraints = new PathConstraints(
                                Units.feetToMeters(18),
                                OnePlusOneAuto.AutoConstants.k_maxAccelerationMetersPerSecondSquared / 2);
        }

        private Swerve m_swerve;
        private Extension m_extension;
        private Collector m_collector;
        private Lift m_lift;
        private HashMap<String, Command> m_eventMap;

        public PickupSideAuto(Swerve p_swerve, Lift p_lift, Collector p_collector, Extension p_extension) {
                m_eventMap = new HashMap<>();

                this.m_collector = p_collector;
                this.m_swerve = p_swerve;
                this.m_lift = p_lift;
                this.m_extension = p_extension;
                AutoHighPoleApproachCommandGroup m_highPoleApproach = new AutoHighPoleApproachCommandGroup(m_extension,
                                m_lift,
                                m_collector);

                DepositSequenceCommandGroup m_depositSequenceCommandGroup = new DepositSequenceCommandGroup(
                                m_lift,
                                m_extension,
                                m_collector);
                IntakeOffCommand m_intakeOff = new IntakeOffCommand(m_collector);

                m_eventMap.put("startingPosition", new RaiseToUnlockRatchetCommand(m_lift)
                                .andThen(m_highPoleApproach)
                                .andThen(m_depositSequenceCommandGroup));

                // m_eventMap.put("startingPosition", new
                // HighPoleApproachCommandGroup(m_extension, m_lift,
                // m_collector).andThen(
                // new DepositSequenceCommandGroup(m_lift, m_extension,
                // m_collector)));

                m_eventMap.put("startToCollect",
                                new InstantCommand(() -> m_collector.m_intakeState = IntakeState.HOLD_IN)
                                                .andThen(new AutoGroundCollectionCommandGroup(m_extension, m_lift,
                                                                m_collector)));
                m_eventMap.put("collectCubePosition",
                                new WaitCommand(1)
                                                .andThen(new InstantCommand(
                                                                () -> m_collector.m_intakeState = IntakeState.OFF))
                                                .andThen(new CarryRetractedCommandGroup(
                                                                m_extension, m_lift, m_collector)));

                m_eventMap.put("turnOffCollectorPoint",
                                new InstantCommand(() -> m_collector.m_intakeState = IntakeState.OFF));
                m_eventMap.put("startToDeposit", new HighPoleApproachCommandGroup(m_extension, m_lift,
                                m_collector));

                m_eventMap.put("depositCubePosition",
                                new DepositSequenceCommandGroup(m_lift, m_extension,
                                                m_collector));

                // m_eventMap.put("depositCubePosition", new InstantCommand(() -> m_lift.m_state
                // = LiftPosition.HIGH_POLE)
                // .andThen(m_highPoleApproach).andThen(m_depositSequenceCommandGroup).andThen(m_intakeOff)
                // .andThen(new WaitCommand(0.4)));

                addRequirements(m_collector, m_swerve, m_lift, m_extension);
                addCommands(m_swerve.getAutoBuilder(m_eventMap).fullAuto(
                                PathPlanner.loadPathGroup("LeftSideAuto",
                                                PickupSideAutoConstants.goOutToCollectConstraints)));
                // addCommands(m_eventMap.get("startingPosition"));
        }

}
