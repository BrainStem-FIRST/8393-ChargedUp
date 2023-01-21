package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

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

    public static final int LEFT_STICK_X_AXIS = 0;
    public static final int LEFT_STICK_Y_AXIS = 1;
    public static final int LEFT_TRIGGER = 2;
    public static final int RIGHT_TRIGGER = 3;
    public static final int RIGHT_STICK_Y_AXIS = 5;
    public static final int RIGHT_STICK_X_AXIS = 4;
    public static final int A_BUTTON = 1;
    public static final int B_BUTTON = 2;
    public static final int X_BUTTON = 3;
    public static final int Y_BUTTON = 4;
    public static final int LEFT_BUMPER = 5;
    public static final int RIGHT_BUMPER = 7;
    public static final int BACK_BUTTON = 8;
    public static final int START_BUTTON = 9;
    public static final int LEFT_JOYSTICK_BUTTON = 10;
    public static final int RIGHT_JOYSTICK_BUTTON = 11;
}
  /* Controllers */
  private final Joystick driver1 = new Joystick(0);
  private final Joystick driver2 = new Joystick(1);

  /* Drive Controls */
  private final int translationAxis = JoystickConstants.LEFT_STICK_Y_AXIS;
  private final int strafeAxis = JoystickConstants.LEFT_STICK_X_AXIS;
  private final int rotationAxis = JoystickConstants.RIGHT_STICK_X_AXIS;

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver1, XboxController.Button.kY.value);
  private final JoystickButton robotCentric = new JoystickButton(driver1, XboxController.Button.kLeftBumper.value);

  /* Subsystems */
  private final Drivetrain drivetrain = new Drivetrain();
  private final Grabber grabber = new Grabber();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Drivetrain 
    drivetrain.setDefaultCommand(
        new TeleopSwerve(
            drivetrain,
            () -> -driver1.getRawAxis(translationAxis),
            () -> -driver1.getRawAxis(strafeAxis),
            () -> -driver1.getRawAxis(rotationAxis),
            () -> robotCentric.getAsBoolean()));

    // Grabber
    


    

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
    zeroGyro.onTrue(new InstantCommand(() -> drivetrain.zeroGyro()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //return new exampleAuto(drivetrain);
    return null;
  }
}