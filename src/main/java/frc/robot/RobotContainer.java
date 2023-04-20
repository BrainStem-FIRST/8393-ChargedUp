package frc.robot;

import java.util.ArrayList;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.util.PathPlannerFlipper;
import frc.robot.autos.*;
import frc.robot.commandGroups.CarryRetractedCommandGroup;
import frc.robot.commandGroups.CollectCommandGroup;
import frc.robot.commandGroups.GroundCollectionCommandGroup;
import frc.robot.commandGroups.GroundRetractedCommandGroup;
import frc.robot.commandGroups.LowPoleApproachCommandGroup;
import frc.robot.commandGroups.ShelfCollectionApproachCommandGroup;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.BrainSTEMSubsystem;
import frc.robot.utilities.ToggleButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public static final class JoystickConstants {
    public static final int k_leftStickXAxis = 0;
    public static final int k_leftStickYAxis = 1;
    public static final int k_leftTrigger = 2;
    public static final int k_rightTrigger = 3;
    public static final int k_rightStickYAxis = 5;
    public static final int k_rightStickXAxis = 4;
    public static final int k_aButton = 1;
    public static final int k_bButton = 2;
    public static final int k_xButton = 3;
    public static final int k_yButton = 4;
    public static final int k_leftBumper = 5;
    public static final int k_rightBumper = 6;
    public static final int k_backButton = 7;
    public static final int k_startButton = 8;
    public static final int k_leftJoystickButton = 9;
    public static final int k_rightJoystickButton = 10;
  }

  /* Controllers */
  public final Joystick m_driver1 = new Joystick(0);
  public final Joystick m_driver2 = new Joystick(1);
  public static final double k_stickDeadband = 0.05;

  /* Drive Controls */
  final int k_translationAxis = XboxController.Axis.kLeftY.value;
  private final int k_strafeAxis = XboxController.Axis.kLeftX.value;
  private final int k_rotationAxis = XboxController.Axis.kRightX.value;

  /* Toggle Buttons */
  ToggleButton driver1X = new ToggleButton();
  ToggleButton driver1A = new ToggleButton();

  /* Driver 1 Buttons */
  public final JoystickButton m_driver1AButton = new JoystickButton(m_driver1, JoystickConstants.k_aButton);
  public final JoystickButton m_driver1BButton = new JoystickButton(m_driver1, JoystickConstants.k_bButton);
  public final JoystickButton m_driver1XButton = new JoystickButton(m_driver1, JoystickConstants.k_xButton);
  public final JoystickButton m_zeroGyro = new JoystickButton(m_driver1, JoystickConstants.k_startButton);
  public final JoystickButton m_driver1LeftBumper = new JoystickButton(m_driver1, JoystickConstants.k_leftBumper);
  public final JoystickButton m_driver1RightBumper = new JoystickButton(m_driver1, JoystickConstants.k_rightBumper);
  public final JoystickButton m_driver1YButton = new JoystickButton(m_driver1, JoystickConstants.k_yButton);

  /* Driver 2 Buttons */
  public final JoystickButton m_driver2AButton = new JoystickButton(m_driver2, JoystickConstants.k_aButton);
  public final JoystickButton m_driver2BButton = new JoystickButton(m_driver2, JoystickConstants.k_bButton);
  public final JoystickButton m_driver2YButton = new JoystickButton(m_driver2, JoystickConstants.k_yButton);
  public final JoystickButton m_driver2XButton = new JoystickButton(m_driver2, JoystickConstants.k_xButton);
  public final JoystickButton m_driver2LeftBumper = new JoystickButton(m_driver2, JoystickConstants.k_leftBumper);
  public final JoystickButton m_driver2RightBumper = new JoystickButton(m_driver2, JoystickConstants.k_rightBumper);

  /* Subsystems */
  Swerve m_swerve = new Swerve();
  Lift m_lift = new Lift();
  Extension m_extension = new Extension();
  Collector m_collector = new Collector();

  /* Command Groups */
  public LowPoleApproachCommandGroup m_lowPoleApproach = new LowPoleApproachCommandGroup(m_extension, m_lift,
      m_collector);
  public GroundRetractedCommandGroup m_groundRetracted = new GroundRetractedCommandGroup(m_extension, m_lift);
  public CarryRetractedCommandGroup m_carryRetracted = new CarryRetractedCommandGroup(m_extension, m_lift, m_collector);
  public CollectCommandGroup m_collectCommandGroup = new CollectCommandGroup(m_collector);
  public GroundCollectionCommandGroup m_groundCollection = new GroundCollectionCommandGroup(m_extension, m_lift,
      m_collector);
  public ShelfCollectionApproachCommandGroup m_shelfCollection = new ShelfCollectionApproachCommandGroup(m_extension,
      m_lift);
  public MonkDrive monkDrive = new MonkDrive(m_swerve, false);
  public DriveUntilLimelightCommand leftGreenMonkDrive = new DriveUntilLimelightCommand(true, m_swerve, () -> false);
  public DriveUntilLimelightCommand rightGreenMonkDrive = new DriveUntilLimelightCommand(false, m_swerve, () -> false);
  public SendableChooser<Command> autoChooser = new SendableChooser<>();

  public AutoCenter m_autoCenter = new AutoCenter(m_swerve, m_lift, m_collector, m_extension);
  public BarrierSideAuto m_barrierSideAuto = new BarrierSideAuto(m_swerve, m_lift, m_collector, m_extension);
  public WireSideAuto m_wireSideAuto = new WireSideAuto(m_swerve, m_lift, m_collector, m_extension);

  // public CANSparkMax m_collectorMotor1 = new CANSparkMax(19,
  // MotorType.kBrushless);
  // public CANSparkMax m_collectorMotor2 = new CANSparkMax(22,
  // MotorType.kBrushless);

  // public DefaultLimelightCommand m_limelightCommand = new
  // DefaultLimelightCommand(m_Limelight);
  /*
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    m_swerve.setDefaultCommand(
        new TeleopSwerve(
            m_swerve,
            () -> -(m_driver1.getRawAxis(k_translationAxis) * m_lift.m_swerveMultiplyerTranslation),
            () -> -(m_driver1.getRawAxis(k_strafeAxis) * m_lift.m_swerveMultiplyerTranslation),
            () -> -(m_driver1.getRawAxis(k_rotationAxis) * m_lift.m_swerveTurningMultiplyer),
            () -> false));

    autoChooser.setDefaultOption("Center Auto", m_autoCenter);
    autoChooser.addOption("Barrier Side Auto", m_barrierSideAuto);
    autoChooser.addOption("Wire Side Auto", m_wireSideAuto);

    // m_limelight.setDefaultCommand(new DefaultLimelightCommand(m_limelight));

    // Configure the button bindings
    configureButtonBindings();

    // Basic Testing and Commands
    ShuffleboardTab commandTab = Shuffleboard.getTab("Commands");
    ShuffleboardTab adjustmentsTab = Shuffleboard.getTab("Adjustments");
    ShuffleboardTab autoAdjustmentsTab = Shuffleboard.getTab("Auto Adjustments");

    commandTab.add("Reset Extension", m_extension.resetExtensionBase()).withSize(2, 1);
    commandTab.add("Reset Modules To Absolute", m_swerve.resetModuleBase().ignoringDisable(true)).withSize(2, 1);

    commandTab.add("Enable lift brake", m_lift.enableBrakeModeBase()).withSize(2, 1);
    commandTab.add("Disable lift brake", m_lift.disableBrakeModeBase()).withSize(2, 1);
    commandTab.add("Enable ext brake", m_extension.enableBrakeModeBase()).withSize(2, 1);
    commandTab.add("Disable ext brake", m_extension.disableBrakeModeBase()).withSize(2, 1);
    commandTab.add("Reset Lift Height", m_lift.resetLiftHeightBase()).withSize(2, 1);


    adjustmentsTab.add("E - Adjust High Forward", new InstantCommand(() -> m_extension.adjustExtensionValue("high", 0.02))).withSize(2, 1);
    adjustmentsTab.add("E - Adjust High Back", new InstantCommand(() -> m_extension.adjustExtensionValue("high", -0.02))).withSize(2, 1);
    adjustmentsTab.add("E - Adjust Low Forward", new InstantCommand(() -> m_extension.adjustExtensionValue("low", 0.02))).withSize(2, 1);
    adjustmentsTab.add("E - Adjust Low Back", new InstantCommand(() -> m_extension.adjustExtensionValue("low", -0.02))).withSize(2, 1);
    adjustmentsTab.add("E - Adjust Shelf Forward", new InstantCommand(() -> m_extension.adjustExtensionValue("low", 0.02))).withSize(2, 1);
    adjustmentsTab.add("E - Adjust Shelf Back", new InstantCommand(() -> m_extension.adjustExtensionValue("low", -0.02))).withSize(2, 1);
    adjustmentsTab.addNumber("E - High %", () -> m_extension.m_adjustableHighMultiplier * 100);
    adjustmentsTab.addNumber("E - Low %", () -> m_extension.m_adjustableLowMultiplier * 100);
    adjustmentsTab.addNumber("E - Shelf %", () -> m_extension.m_adjustableShelfCollectionMultiplier * 100);

    adjustmentsTab.add("L - Adjust High Up", m_lift.adjustLiftValue("high", 0.02)).withSize(2, 1);
    adjustmentsTab.add("L - Adjust High Down", m_lift.adjustLiftValue("high", -0.02)).withSize(2, 1);
    adjustmentsTab.add("L - Adjust Low Up", m_lift.adjustLiftValue("low", 0.02)).withSize(2, 1);
    adjustmentsTab.add("L - Adjust Low Down", m_lift.adjustLiftValue("low", -0.02)).withSize(2, 1);
    adjustmentsTab.add("L - Adjust Shelf Up", m_lift.adjustLiftValue("low", 0.02)).withSize(2, 1);
    adjustmentsTab.add("L - Adjust Shelf Down", m_lift.adjustLiftValue("low", -0.02)).withSize(2, 1);
    adjustmentsTab.addNumber("L - High %", () -> m_lift.m_adjustableHighMultiplier * 100);
    adjustmentsTab.addNumber("L - Low %", () -> m_lift.m_adjustableLowMultiplier * 100);
    adjustmentsTab.addNumber("L - Shelf %", () -> m_lift.m_adjustableShelfCollectionMultiplier * 100);

    autoAdjustmentsTab.add("Center Auto Further Foward", new InstantCommand(() -> m_autoCenter.adjustGoOntoChargeStationTrajectory(-0.02))).withSize(2, 1);
    autoAdjustmentsTab.add("Center Auto Further Back", new InstantCommand(() -> m_autoCenter.adjustGoOntoChargeStationTrajectory(0.02))).withSize(2, 1);
    autoAdjustmentsTab.add("High Pole Deposit Adjustment Up", new InstantCommand(() -> m_lift.m_adjustableAutoDepositMultiplier += 0.02)).withSize(2, 1);
    autoAdjustmentsTab.add("High Pole Deposit Adjustment Down", new InstantCommand(() -> m_lift.m_adjustableAutoDepositMultiplier -= 0.02)).withSize(2, 1);
    autoAdjustmentsTab.addNumber("High Pole Deposit %", () -> m_lift.m_adjustableAutoDepositMultiplier);
    autoAdjustmentsTab.addNumber("Center Auto Charge Station Value ", () -> m_autoCenter.m_adjustableGoBackOntoChargeStationValue);




    SmartDashboard.putNumber("S - Turning ", m_lift.m_swerveTurningMultiplyer);
    SmartDashboard.putNumber("S - Translation ", m_lift.m_swerveMultiplyerTranslation);

    Field2d trajTest = new Field2d();
    PathPlannerTrajectory testTraj = PathPlanner.generatePath(
        new PathConstraints(1.0, 1.0),
        new PathPoint(new Translation2d(), new Rotation2d()),
        new PathPoint(new Translation2d(1.0, 1.0), new Rotation2d()));
    trajTest.getObject("Original traj").setTrajectory(testTraj);
    trajTest.getObject("Flipped traj")
        .setTrajectory(PathPlannerFlipper.flipTrajectory(testTraj, new PathConstraints(1.0, 1.0)));

    SmartDashboard.putData("Traj Field", trajTest);
    SmartDashboard.putData(autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public ArrayList<BrainSTEMSubsystem> getBrainSTEMSubsystems() {
    ArrayList<BrainSTEMSubsystem> brainSTEMSubsystems = new ArrayList<>();
    brainSTEMSubsystems.add(m_lift);
    brainSTEMSubsystems.add(m_extension);
    brainSTEMSubsystems.add(m_swerve);
    brainSTEMSubsystems.add(m_collector);
    return brainSTEMSubsystems;
  }

}
