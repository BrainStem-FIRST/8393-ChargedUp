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
import frc.robot.Robot.RobotMode;
import frc.robot.autos.*;
import frc.robot.commandGroups.CarryRetractedCommandGroup;
import frc.robot.commandGroups.CollectCommandGroup;
import frc.robot.commandGroups.GroundRetractedCommandGroup;
import frc.robot.commandGroups.LowPoleApproachCommandGroup;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Collector.CollectorState;
import frc.robot.subsystems.Collector.IntakeState;
import frc.robot.subsystems.Lift.LiftPosition;
import frc.robot.utilities.BrainSTEMSubsystem;

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
    public static final double k_stickDeadband = 0.1;
  
  /* Drive Controls */
    final int k_translationAxis = XboxController.Axis.kLeftY.value;
    private final int k_strafeAxis = XboxController.Axis.kLeftX.value;
    private final int k_rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver 1 Buttons */
    private final JoystickButton m_driver1AButton = new JoystickButton(m_driver1, JoystickConstants.k_aButton);
    private final JoystickButton m_driver1BButton = new JoystickButton(m_driver1, JoystickConstants.k_bButton);
    private final JoystickButton m_driver1XButton = new JoystickButton(m_driver1, JoystickConstants.k_xButton);
    private final JoystickButton m_zeroGyro = new JoystickButton(m_driver1, JoystickConstants.k_startButton);
    public final JoystickButton m_driver1LeftBumper = new JoystickButton(m_driver1, JoystickConstants.k_leftBumper);
    public final JoystickButton m_driver1RightBumper = new JoystickButton(m_driver1, JoystickConstants.k_rightBumper);
    private final JoystickButton m_driver1YButton = new JoystickButton(m_driver1, JoystickConstants.k_yButton);

  /* Driver 2 Buttons */
    private final JoystickButton m_driver2AButton = new JoystickButton(m_driver2, JoystickConstants.k_aButton);
    private final JoystickButton m_driver2BButton = new JoystickButton(m_driver2, JoystickConstants.k_bButton);
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
    public LowPoleApproachCommandGroup m_lowPoleApproach = new LowPoleApproachCommandGroup(m_extension, m_lift);
    public GroundRetractedCommandGroup m_groundRetracted = new GroundRetractedCommandGroup(m_extension, m_lift);
    public CarryRetractedCommandGroup m_carryRetracted = new CarryRetractedCommandGroup(m_extension, m_lift);
    public CollectCommandGroup m_collectCommandGroup = new CollectCommandGroup(m_collector);


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

    //mextension.setDefaultCommand(new DefaultExtensionCommand(mextension, () -> -driver1.getRawAxis(JoystickConstants.RIGHT_TRIGGER)));

    
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
  /* Driver Buttons */
  if(Robot.s_robotMode == RobotMode.DEPOSITING) {
    //m_driver1AButton.toggleOnTrue(new InstantCommand(() -> m_carryRetracted.schedule()));
    // m_driver1XButton.toggleOnTrue(new InstantCommand(() -> m_lowPoleApproach.schedule()));
  } else {
    m_driver1RightBumper.whileTrue(new InstantCommand(() -> m_collectCommandGroup.schedule()));
  }

    m_driver1LeftBumper.toggleOnTrue(new InstantCommand(() -> m_collector.m_collectorState = CollectorState.OPEN));
    
    //m_driver1RightBumper.toggleOnTrue(new InstantCommand(() -> m_collector.m_intakeState = IntakeState.OFF));


    
    
    m_driver2AButton.onTrue(new InstantCommand(() -> Robot.s_robotMode = RobotMode.COLLECTING));
    m_driver2BButton.onTrue(new InstantCommand(() -> Robot.s_robotMode = RobotMode.DEPOSITING));
    //m_driver2XButton.toggleOnTrue(new InstantCommand(() -> m_extension.scheduleLowPole()));
    //m_driver2YButton.toggleOnTrue(new InstantCommand(() -> m_extension.scheduleHighPole()));

    m_driver2LeftBumper.toggleOnTrue(new InstantCommand(() -> m_lift.decrementLiftHeight()));
    m_driver2RightBumper.toggleOnTrue(new InstantCommand(() -> m_lift.incrementLiftHeight()));

    m_zeroGyro.whileTrue(new InstantCommand(() -> m_swerve.zeroGyro()));

    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
  
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    //return new SequentialCommandGroup(new AutoDepositCommand(m_collector, m_extension, 3), new exampleAuto(m_swerve));
    return new SequentialCommandGroup(new autoCenter(m_swerve, m_lift, m_collector));
    
    // An ExampleCommand will run in autonomous
    //return new InstantCommand(grabber::collectorOn);
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
