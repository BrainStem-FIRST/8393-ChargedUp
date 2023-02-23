package frc.robot;

import java.util.ArrayList;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
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
    public static final int k_rightBumper = 7;
    public static final int k_backButton = 8;
    public static final int k_startButton = 9;
    public static final int k_leftJoystickButton = 10;
    public static final int k_rightJoystickButton = 11;
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
  private final JoystickButton m_extensionOn = new JoystickButton(m_driver1, JoystickConstants.k_rightTrigger);
  public final JoystickButton m_liftUp = new JoystickButton(m_driver1, JoystickConstants.k_yButton);
  public final JoystickButton m_liftDown = new JoystickButton(m_driver1, JoystickConstants.k_bButton); 
  private final JoystickButton m_liftStop = new JoystickButton(m_driver1, JoystickConstants.k_xButton);
  private final JoystickButton m_collectorOn = new JoystickButton(m_driver1, JoystickConstants.k_rightBumper);
  private final JoystickButton m_collectorOut = new JoystickButton(m_driver1, JoystickConstants.k_leftBumper);
  private final JoystickButton m_zeroGyro = new JoystickButton(m_driver1, JoystickConstants.k_yButton);
  private final JoystickButton m_robotCentric = new JoystickButton(m_driver1, JoystickConstants.k_leftBumper);

  /* Driver 2 Buttons */
  private final JoystickButton m_retracted = new JoystickButton(m_driver2, JoystickConstants.k_aButton);
  private final JoystickButton m_collection = new JoystickButton(m_driver2, JoystickConstants.k_bButton);
  public final JoystickButton m_highPole = new JoystickButton(m_driver2, JoystickConstants.k_yButton);
  public final JoystickButton m_lowPole = new JoystickButton(m_driver2, JoystickConstants.k_xButton);
  public final JoystickButton m_collectorRun = new JoystickButton(m_driver2, JoystickConstants.k_leftBumper);
  public final JoystickButton m_collectorClose = new JoystickButton(m_driver2, JoystickConstants.k_rightBumper);

    /* Subsystems */
  Swerve m_swerve = new Swerve();
  Lift m_lift = new Lift();
  Extension m_extension = new Extension();
  Collector m_collector = new Collector();
  

  /* Commands */

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    
    m_swerve.setDefaultCommand(
        new TeleopSwerve(
            m_swerve,
            () -> -m_driver1.getRawAxis(k_translationAxis) * 0.5,
            () -> -m_driver1.getRawAxis(k_strafeAxis) * 0.5,
            () -> -m_driver1.getRawAxis(k_rotationAxis) * 0.5,
            () -> m_robotCentric.getAsBoolean()));

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
 
    m_liftUp.whileTrue(new InstantCommand(() -> m_lift.state = LiftPosition.UP));
    m_liftDown.whileTrue(new InstantCommand(() -> m_lift.state = LiftPosition.DOWN));




    // liftStop.whileTrue(new InstantCommand(() -> mlift.state = LiftPosition.STOP));
    // resetEncoders.whileTrue(new InstantCommand(mlift::resetLiftEncoder));
    // extensionRatchet.whileTrue(new InstantCommand(() -> mextension.ratchetState = RatchetPosition.DISENGAGED));
    // extensionRatchet.whileFalse(new InstantCommand(() -> mextension.ratchetState = RatchetPosition.ENGAGED));
    
    m_retracted.toggleOnTrue(new InstantCommand(() -> m_extension.scheduleRetracted()));
    m_collection.toggleOnTrue(new InstantCommand(() -> m_extension.scheduleCollection()));
    m_lowPole.toggleOnTrue(new InstantCommand(() -> m_extension.scheduleLowPole()));
    m_highPole.toggleOnTrue(new InstantCommand(() -> m_extension.scheduleHighPole()));
    // extend.whileTrue(new InstantCommand(() -> SmartDashboard.putBoolean("High Not Being Called", false)));
    // extend.whileFalse(new InstantCommand(() -> SmartDashboard.putBoolean("High Not Being Called", true)));

    // retract.whileTrue(new InstantCommand(() -> SmartDashboard.putBoolean("Retract Not Being Called", false)));
    // retract.whileFalse(new InstantCommand(() -> SmartDashboard.putBoolean("Retract Not Being Called", true)));

    m_collectorRun.toggleOnTrue(new InstantCommand(() -> m_collector.toggleIntakeState()));
    // collectorOut.whileTrue(new InstantCommand(() -> m_collector.intakeState = IntakeState.OUT));
    m_collectorRun.toggleOnFalse(new InstantCommand(() -> m_collector.toggleIntakeButton()));

    // collectorClose.whileTrue(new InstantCommand(() -> m_collector.collectorState = CollectorState.CLOSED));
    // collectorClose.whileFalse(new InstantCommand(() -> m_collector.collectorState = CollectorState.OFF));

    m_collectorClose.toggleOnTrue(new InstantCommand(() -> m_collector.toggleClawState()));
    m_collectorClose.toggleOnFalse(new InstantCommand(() -> m_collector.toggleClawButton()));

    m_zeroGyro.whileTrue(new InstantCommand(() -> m_swerve.zeroGyro()));
    // spinNeoMotor.whileTrue(new InstantCommand(() -> mgrabber.collectorOn()));
    // spinNeoMotor.onFalse(new InstantCommand(() -> mgrabber.collectorOff()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
  
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //return new InstantCommand(grabber::collectorOn);
    return new exampleAuto(m_swerve);
  }

  public ArrayList<BrainSTEMSubsystem> getBrainSTEMSubsystems(){
    ArrayList<BrainSTEMSubsystem> brainSTEMSubsystems = new ArrayList<>();
    brainSTEMSubsystems.add(m_lift);
    brainSTEMSubsystems.add(m_extension);
    brainSTEMSubsystems.add(m_swerve);


    return brainSTEMSubsystems;
  }
}
