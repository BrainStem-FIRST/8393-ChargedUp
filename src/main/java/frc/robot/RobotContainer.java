package frc.robot;

import java.util.ArrayList;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.*;
import frc.robot.commandGroups.CarryRetractedCommandGroup;
import frc.robot.commandGroups.CollectCommandGroup;
import frc.robot.commandGroups.GroundCollectionCommandGroup;
import frc.robot.commandGroups.GroundRetractedCommandGroup;
import frc.robot.commandGroups.LowPoleApproachCommandGroup;
import frc.robot.commandGroups.ShelfCollectionApproachCommandGroup;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Collector.CollectorState;
import frc.robot.subsystems.Collector.IntakeState;
import frc.robot.subsystems.Lift.LiftPosition;
import frc.robot.utilities.BrainSTEMSubsystem;
import frc.robot.utilities.ToggleButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
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
    private final Joystick m_driver2 = new Joystick(1);
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
    //Limelight m_limelight = new Limelight();

    /* Command Groups */
    public LowPoleApproachCommandGroup m_lowPoleApproach = new LowPoleApproachCommandGroup(m_extension, m_lift, m_collector);
    public GroundRetractedCommandGroup m_groundRetracted = new GroundRetractedCommandGroup(m_extension, m_lift);
    public CarryRetractedCommandGroup m_carryRetracted = new CarryRetractedCommandGroup(m_extension, m_lift, m_collector);
    public CollectCommandGroup m_collectCommandGroup = new CollectCommandGroup(m_collector);
    public GroundCollectionCommandGroup m_groundCollection = new GroundCollectionCommandGroup(m_extension, m_lift, m_collector);
    public ShelfCollectionApproachCommandGroup m_shelfCollection = new ShelfCollectionApproachCommandGroup(m_extension, m_lift);
    public MonkDrive monkDrive = new MonkDrive(m_swerve);
    public GreenMonkDrive leftGreenMonkDrive = new GreenMonkDrive(m_swerve, true);
    public GreenMonkDrive rightGreenMonkDrive = new GreenMonkDrive(m_swerve, false);
    //public DefaultLimelightCommand m_limelightCommand = new DefaultLimelightCommand(m_Limelight);
  /*
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    
    m_swerve.setDefaultCommand(
        new TeleopSwerve(
            m_swerve,
            () -> -(m_driver1.getRawAxis(k_translationAxis) * Math.abs(m_driver1.getRawAxis(k_translationAxis))),
            () -> -(m_driver1.getRawAxis(k_strafeAxis) * Math.abs(m_driver1.getRawAxis(k_strafeAxis))),
            () -> -(m_driver1.getRawAxis(k_rotationAxis) * Math.abs(m_driver1.getRawAxis(k_rotationAxis)) * 1.2),
            () -> false));

    //m_limelight.setDefaultCommand(new DefaultLimelightCommand(m_limelight));

    // Configure the button bindings
    configureButtonBindings();
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
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //return new autoSide(m_swerve, m_lift, m_collector, m_extension);
    return new AutoCenter(m_swerve, m_lift, m_collector, m_extension);
  }

  public ArrayList<BrainSTEMSubsystem> getBrainSTEMSubsystems(){
    ArrayList<BrainSTEMSubsystem> brainSTEMSubsystems = new ArrayList<>();
    brainSTEMSubsystems.add(m_lift);
    brainSTEMSubsystems.add(m_extension);
    brainSTEMSubsystems.add(m_swerve);
    brainSTEMSubsystems.add(m_collector);
    return brainSTEMSubsystems;
  }

  
}
