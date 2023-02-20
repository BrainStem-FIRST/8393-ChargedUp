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
    /* Controllers */
    private final Joystick driver1 = new Joystick(0);
    private final Joystick driver2 = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  public final JoystickButton liftUp = new JoystickButton(driver1, XboxController.Button.kY.value);
  public final JoystickButton liftDown = new JoystickButton(driver1, XboxController.Button.kB.value); 
  private final JoystickButton liftStop = new JoystickButton(driver1, XboxController.Button.kX.value);
  private final JoystickButton resetEncoders = new JoystickButton(driver1, XboxController.Button.kRightBumper.value);
  private final JoystickButton servoToMin = new JoystickButton(driver1, XboxController.Button.kLeftBumper.value);
  private final JoystickButton zeroGyro = new JoystickButton(driver1, XboxController.Button.kY.value);
  private final JoystickButton robotCentric = new JoystickButton(driver1, XboxController.Button.kLeftBumper.value);
  private final JoystickButton spinNeoMotor = new JoystickButton(driver1, XboxController.Button.kA.value);
    /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final Grabber mgrabber = new Grabber();
  public final Lift mlift = new Lift();
  public Extension mextension = new Extension();
  
  

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -driver1.getRawAxis(translationAxis) * 0.5,
            () -> -driver1.getRawAxis(strafeAxis) * 0.5,
            () -> -driver1.getRawAxis(rotationAxis) * 0.5,
            () -> robotCentric.getAsBoolean()));

    
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
    resetEncoders.whileTrue(new InstantCommand(mlift::resetLiftEncoder));
    servoToMin.whileTrue(new InstantCommand(mextension::moveServoToMin));
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
