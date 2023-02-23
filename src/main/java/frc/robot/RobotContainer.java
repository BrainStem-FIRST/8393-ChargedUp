package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.JoystickConstants;
import frc.robot.autos.*;
import frc.robot.commandGroups.CollectionExtensionCommandGroup;
import frc.robot.commandGroups.HighPoleExtensionCommandGroup;
import frc.robot.commandGroups.LowPoleExtensionCommandGroup;
import frc.robot.commandGroups.RetractedExtensionCommandGroup;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Collector.CollectorState;
import frc.robot.subsystems.Collector.IntakeState;
import frc.robot.subsystems.Extension.RatchetPosition;
import frc.robot.subsystems.Extension.TelescopePosition;
import frc.robot.subsystems.Lift.LiftPosition;
import frc.robot.utilities.BrainSTEMSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    public final Joystick driver1 = new Joystick(0);
    private final Joystick driver2 = new Joystick(1);

    /* Drive Controls */
    final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver 1 Buttons */
  private final JoystickButton extensionOn = new JoystickButton(driver1, JoystickConstants.RIGHT_TRIGGER);
  public final JoystickButton liftUp = new JoystickButton(driver1, XboxController.Button.kY.value);
  public final JoystickButton liftDown = new JoystickButton(driver1, XboxController.Button.kB.value); 
  private final JoystickButton liftStop = new JoystickButton(driver1, XboxController.Button.kX.value);
  private final JoystickButton collectorOn = new JoystickButton(driver1, XboxController.Button.kRightBumper.value);
  private final JoystickButton collectorOut = new JoystickButton(driver1, XboxController.Button.kLeftBumper.value);
  private final JoystickButton zeroGyro = new JoystickButton(driver1, XboxController.Button.kY.value);
  private final JoystickButton robotCentric = new JoystickButton(driver1, XboxController.Button.kLeftBumper.value);

  /* Driver 2 Buttons */
  private final JoystickButton retracted = new JoystickButton(driver2, XboxController.Button.kA.value);
  private final JoystickButton collection = new JoystickButton(driver2, XboxController.Button.kB.value);
  public final JoystickButton highPole = new JoystickButton(driver2, XboxController.Button.kY.value);
  public final JoystickButton lowPole = new JoystickButton(driver2, XboxController.Button.kX.value);
  public final JoystickButton collectorRun = new JoystickButton(driver2, XboxController.Button.kLeftBumper.value);
  public final JoystickButton collectorClose = new JoystickButton(driver2, XboxController.Button.kRightBumper.value);

    /* Subsystems */
  Swerve s_Swerve = new Swerve();
  Lift mlift = new Lift();
  Extension mextension = new Extension();
  Collector m_collector = new Collector();
  

  /* Commands */
  private HighPoleExtensionCommandGroup extendHigh;
  private LowPoleExtensionCommandGroup extendLow;
  private RetractedExtensionCommandGroup extendRetract;
  private CollectionExtensionCommandGroup extendCollect;



  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    
    extendHigh = new HighPoleExtensionCommandGroup(mextension);
    extendLow = new LowPoleExtensionCommandGroup(mextension);
    extendCollect = new CollectionExtensionCommandGroup(mextension);
    extendRetract = new RetractedExtensionCommandGroup(mextension);
    
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -driver1.getRawAxis(translationAxis) * 0.5,
            () -> -driver1.getRawAxis(strafeAxis) * 0.5,
            () -> -driver1.getRawAxis(rotationAxis) * 0.5,
            () -> robotCentric.getAsBoolean()));

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
 
    liftUp.whileTrue(new InstantCommand(() -> mlift.state = LiftPosition.UP));
    liftDown.whileTrue(new InstantCommand(() -> mlift.state = LiftPosition.DOWN));




    // liftStop.whileTrue(new InstantCommand(() -> mlift.state = LiftPosition.STOP));
    // resetEncoders.whileTrue(new InstantCommand(mlift::resetLiftEncoder));
    // extensionRatchet.whileTrue(new InstantCommand(() -> mextension.ratchetState = RatchetPosition.DISENGAGED));
    // extensionRatchet.whileFalse(new InstantCommand(() -> mextension.ratchetState = RatchetPosition.ENGAGED));
    
    retracted.toggleOnTrue(new InstantCommand(() -> mextension.scheduleRetracted()));
    collection.toggleOnTrue(new InstantCommand(() -> mextension.scheduleCollection()));
    lowPole.toggleOnTrue(new InstantCommand(() -> mextension.scheduleLowPole()));
    highPole.toggleOnTrue(new InstantCommand(() -> mextension.scheduleHighPole()));
    // extend.whileTrue(new InstantCommand(() -> SmartDashboard.putBoolean("High Not Being Called", false)));
    // extend.whileFalse(new InstantCommand(() -> SmartDashboard.putBoolean("High Not Being Called", true)));

    // retract.whileTrue(new InstantCommand(() -> SmartDashboard.putBoolean("Retract Not Being Called", false)));
    // retract.whileFalse(new InstantCommand(() -> SmartDashboard.putBoolean("Retract Not Being Called", true)));

    collectorRun.whileTrue(new InstantCommand(() -> m_collector.intakeState = IntakeState.IN));
    // collectorOut.whileTrue(new InstantCommand(() -> m_collector.intakeState = IntakeState.OUT));
    collectorRun.whileFalse(new InstantCommand(() -> m_collector.intakeState = IntakeState.OFF));

    // collectorClose.whileTrue(new InstantCommand(() -> m_collector.collectorState = CollectorState.CLOSED));
    // collectorClose.whileFalse(new InstantCommand(() -> m_collector.collectorState = CollectorState.OFF));

    collectorClose.toggleOnTrue(new InstantCommand(() -> m_collector.toggleClawState()));
    collectorClose.toggleOnFalse(new InstantCommand(() -> m_collector.toggleClawButton()));

    zeroGyro.whileTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
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
    return new exampleAuto(s_Swerve);
  }

  public ArrayList<BrainSTEMSubsystem> getBrainSTEMSubsystems(){
    ArrayList<BrainSTEMSubsystem> brainSTEMSubsystems = new ArrayList<>();
    brainSTEMSubsystems.add(mlift);
    brainSTEMSubsystems.add(mextension);
    brainSTEMSubsystems.add(s_Swerve);


    return brainSTEMSubsystems;
  }
}
