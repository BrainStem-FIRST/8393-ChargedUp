// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.cscore.CameraServerJNI.TelemetryKind;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer.JoystickConstants;
import frc.robot.autos.AutoCenter;
import frc.robot.commandGroups.CarryRetractedCommandGroup;
import frc.robot.commandGroups.DepositSequenceCommandGroup;
import frc.robot.commandGroups.DepositSequenceHighPoleCommandGroup;
import frc.robot.commandGroups.GroundCollectionCommandGroup;
import frc.robot.commandGroups.GroundCollectionSequenceCommandGroup;
import frc.robot.commandGroups.HighPoleApproachCommandGroup;
import frc.robot.commandGroups.LowPoleApproachCommandGroup;
import frc.robot.commandGroups.ShelfCarryRetractedCommandGroup;
import frc.robot.commandGroups.ShelfCollectionApproachCommandGroup;
import frc.robot.commands.AlignWheels;
import frc.robot.commands.GreenMonkDrive;
import frc.robot.commands.extensionCommands.ExtensionDepositSequenceCommand;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Collector.CollectorConstants;
import frc.robot.subsystems.Collector.CollectorState;
import frc.robot.subsystems.Collector.IntakeState;
import frc.robot.subsystems.Extension.TelescopePosition;
import frc.robot.subsystems.Lift.HookState;
import frc.robot.subsystems.Lift.LiftPosition;
import frc.robot.utilities.BrainSTEMSubsystem;
import frc.robot.utilities.LimelightHelpers;
import frc.robot.utilities.RunOnce;
import frc.robot.utilities.ToggleButton;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public enum RobotMode {
    COLLECTING,
    DEPOSITING
  }

  public enum DepositLocation {
    HIGH,
    LOW,
    GROUND
  }

  public static RobotMode s_robotMode = RobotMode.COLLECTING;

  public static DepositLocation s_depositLocation = DepositLocation.LOW;

  private ToggleButton m_driver1_A = new ToggleButton();
  private ToggleButton m_driver1_X = new ToggleButton();
  private ToggleButton m_driver1_Y = new ToggleButton();
  private ToggleButton m_driver1_BackButton = new ToggleButton();
  private ToggleButton m_driver1_B = new ToggleButton();

  private ToggleButton m_driver2_A = new ToggleButton();
  private ToggleButton m_driver2_B = new ToggleButton();

  public static CTREConfigs ctreConfigs;

  private Command m_autonomousCommand;

  public static boolean depositSequenceContinue = true;

  private RobotContainer m_robotContainer;

  private boolean hasGroundCollectionRun = false;

  public static boolean hasShelfCarryRetractedRun = false;

  public boolean hasCarryRetractedRun = false;

  private boolean hasDepositingRun = false;
  public ArrayList<BrainSTEMSubsystem> brainSTEMSubsystemsWithoutSwerve;
  private boolean hasMonkDriveCanceled = true;
  private boolean hasLimelightMonkDriveCanceled = true;

  private boolean hasHighPoleApproached = false;
  private boolean hasLowPoleApproached = false;
  private boolean returnedCarryFromHigh = false;
  private boolean returnedCarryFromLow = false;
  public static double limelightCurrent;

  // private boolean hasLimelightRun = false;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    ctreConfigs = new CTREConfigs();
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    brainSTEMSubsystemsWithoutSwerve = m_robotContainer.getBrainSTEMSubsystems();
    brainSTEMSubsystemsWithoutSwerve.remove(m_robotContainer.m_swerve);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.

    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    ArrayList<BrainSTEMSubsystem> brainSTEMSubsystems = m_robotContainer.getBrainSTEMSubsystems();
    for (BrainSTEMSubsystem isubsystem : brainSTEMSubsystems) {
      isubsystem.disablePeriodic();
    }

    // m_robotContainer.m_swerve.resetModuleBase().schedule();
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    m_robotContainer.m_lift.resetLiftEncoder();
    m_robotContainer.m_lift.liftRawPower(0.09);
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    ArrayList<BrainSTEMSubsystem> brainSTEMSubsystems = m_robotContainer.getBrainSTEMSubsystems();
    for (BrainSTEMSubsystem isubsystem : brainSTEMSubsystems) {
      isubsystem.initialize();
    }

    // m_robotContainer.m_lift.m_state = LiftPosition.CARRY;
    m_robotContainer.m_lift.m_state = LiftPosition.RATCHET;
    // m_robotContainer.m_swerve.resetModuleBase().schedule();

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // if((m_robotContainer.m_swerve.getTilt() > 5) ||
    // (m_robotContainer.m_swerve.getTilt() < -5)) {
    // AutoCenter.sideAuto = false;
    // }

    SmartDashboard.putNumber("Limelight X Value", LimelightHelpers.getTX("limelight-b"));
    SmartDashboard.putNumber("Limelight Y Value", LimelightHelpers.getTY("limelight-b"));

  }

  @Override
  public void teleopInit() {
    // hasLimelightRun = false;

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    ArrayList<BrainSTEMSubsystem> brainSTEMSubsystems = m_robotContainer.getBrainSTEMSubsystems();
    for (BrainSTEMSubsystem isubsystem : brainSTEMSubsystems) {
      isubsystem.initialize();
    }

    m_robotContainer.m_lift.m_state = LiftPosition.CARRY;
    m_robotContainer.m_swerve.resetModuleBase().schedule();
  }

  private void setToggleButtons() {

    // m_driver1_A.update(m_robotContainer.m_driver1AButton.getAsBoolean() && !);
    // m_driver1_X.update(m_robotContainer.m_driver1XButton.getAsBoolean() &&
    // !m_robotContainer.m_driver1AButton.getAsBoolean());
    m_driver1_A.update(m_robotContainer.m_driver1AButton.getAsBoolean());
    m_driver1_X.update(m_robotContainer.m_driver1XButton.getAsBoolean());
    m_driver1_Y.update(m_robotContainer.m_driver1YButton.getAsBoolean());
    // m_driver1_B.update(m_robotContainer.m_driver1BButton.getAsBoolean());
    if (m_robotContainer.m_driver1AButton.getAsBoolean()) {
      m_driver1_X.setState(false);
    } else if (m_robotContainer.m_driver1XButton.getAsBoolean()) {
      m_driver1_A.setState(false);
    }
  }

  private void setRobotState() {
    if (m_robotContainer.m_driver2BButton.getAsBoolean()) {
      m_driver1_X.setState(false);
      m_driver1_A.setState(false);
      m_driver1_Y.setState(false);
      m_robotContainer.m_collector.m_collectorState = CollectorState.CLOSED;
      s_robotMode = RobotMode.COLLECTING;
    } else if (m_robotContainer.m_driver2XButton.getAsBoolean()) {
      hasHighPoleApproached = false;
      hasLowPoleApproached = false;
      returnedCarryFromHigh = true;
      returnedCarryFromLow = true;
      m_driver1_X.setState(false);
      m_driver1_A.setState(false);
      m_driver1_Y.setState(false);
      s_robotMode = RobotMode.DEPOSITING;
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    if (m_robotContainer.m_driver2AButton.getAsBoolean()) {
      new InstantCommand(() -> m_robotContainer.m_collector.m_adjustableWheelHoldingPower = 0).schedule();
      new InstantCommand(
          () -> m_robotContainer.m_collector.m_adjustableWheelMotorPower = CollectorConstants.k_wheelMotorSpeed / 2)
          .schedule();
    } else if (m_robotContainer.m_driver2YButton.getAsBoolean()) {
      new InstantCommand(
          () -> m_robotContainer.m_collector.m_adjustableWheelHoldingPower = CollectorConstants.k_wheelMotorHoldingSpeed)
          .schedule();
      new InstantCommand(
          () -> m_robotContainer.m_collector.m_adjustableWheelMotorPower = CollectorConstants.k_wheelMotorSpeed)
          .schedule();
    }

    // if (m_robotContainer.m_driver2YButton.getAsBoolean()) {
    // new InstantCommand(() ->
    // m_robotContainer.m_lift.liftRawPower(-0.085)).schedule();
    // } else {
    // new InstantCommand(() ->
    // m_robotContainer.m_lift.liftRawPower(-0.05)).schedule();
    // }

    // LIME LIGHT
    // ////////////////////////////////////////////////////////////////////////
    SmartDashboard.putNumber("Limelight Light Current ", limelightCurrent);
    limelightCurrent = LimelightHelpers.getTX("limelight");

    // if(m_robotContainer.m_driver2XButton.getAsBoolean()) {
    // new AlignWheels(m_robotContainer.m_swerve).schedule();
    // }

    // MONK DRIVE & OTHER STUFF?
    // //////////////////////////////////////////////////////////
    if (m_robotContainer.m_driver1BButton.getAsBoolean()) {
      // new InstantCommand(() ->
      // m_robotContainer.m_lift.liftRawPower(-0.15)).schedule();
      m_robotContainer.monkDrive.schedule();
      hasMonkDriveCanceled = false;
    } else if (!hasMonkDriveCanceled) {
      // new InstantCommand(() ->
      // m_robotContainer.m_lift.liftRawPower(0.0)).schedule();
      m_robotContainer.monkDrive.cancel();
      hasMonkDriveCanceled = true;
    }

    // LIME LIGHT STRAFE
    // //////////////////////////////////////////////////////////////////
    if (m_robotContainer.m_driver1.getPOV() == 90) {
      m_robotContainer.rightGreenMonkDrive.schedule();
    } else if (m_robotContainer.m_driver1.getPOV() == 270) {
      m_robotContainer.leftGreenMonkDrive.schedule();
    } else {
      m_robotContainer.rightGreenMonkDrive.cancel();
      m_robotContainer.leftGreenMonkDrive.cancel();
    }

    m_driver1_BackButton.update(m_robotContainer.m_driver1.getRawButton(JoystickConstants.k_backButton));
    setRobotState();
    setToggleButtons();

    // EMERGENCY STOP
    // ////////////////////////////////////////////////////////////////////////
    if (m_driver1_BackButton.getState()) {
      for (BrainSTEMSubsystem i_subsystem : brainSTEMSubsystemsWithoutSwerve) {
        i_subsystem.disablePeriodic();
      }
      m_robotContainer.m_lift.turnOffLiftMotors();
      m_robotContainer.m_extension.turnOffExtensionMotor();
      m_robotContainer.m_collector.killCollectorMotors();
    } else {
      for (BrainSTEMSubsystem i_subsystem : brainSTEMSubsystemsWithoutSwerve) {
        i_subsystem.enablePeriodic();
      }
    }

    // DRIVER BUTTON
    // ////////////////////////////////////////////////////////////////////////

    if (m_robotContainer.m_driver1.getRawAxis(JoystickConstants.k_rightTrigger) < 0.7) {
      hasDepositingRun = false;
    }

    if (s_robotMode == RobotMode.DEPOSITING) {

      if (m_robotContainer.m_driver1.getRawAxis(JoystickConstants.k_rightTrigger) > 0.7) {
        hasDepositingRun = true;
        new DepositSequenceCommandGroup(m_robotContainer.m_lift, m_robotContainer.m_extension,
            m_robotContainer.m_collector).schedule();
      }

      if (m_driver1_X.getState() && !hasLowPoleApproached) {
        new LowPoleApproachCommandGroup(m_robotContainer.m_extension, m_robotContainer.m_lift,
            m_robotContainer.m_collector).schedule();
        hasLowPoleApproached = true;
        returnedCarryFromLow = false;
      } else if (!m_driver1_X.getState() && !returnedCarryFromLow) {
        new SequentialCommandGroup(
            new ShelfCarryRetractedCommandGroup(m_robotContainer.m_extension, m_robotContainer.m_lift,
                m_robotContainer.m_collector),
            new InstantCommand(() -> m_robotContainer.m_lift.lowerLiftToCarry()),
            new InstantCommand(() -> m_robotContainer.m_collector.m_intakeState = IntakeState.OFF))
            .schedule();

        returnedCarryFromLow = true;
        hasLowPoleApproached = false;
      }

      if (m_driver1_Y.getState() && !hasHighPoleApproached) {
        new HighPoleApproachCommandGroup(m_robotContainer.m_extension, m_robotContainer.m_lift,
            m_robotContainer.m_collector).schedule();
        hasHighPoleApproached = true;
        returnedCarryFromHigh = false;
      } else if (!m_driver1_Y.getState() && !returnedCarryFromHigh) {
        new SequentialCommandGroup(
            new ShelfCarryRetractedCommandGroup(m_robotContainer.m_extension, m_robotContainer.m_lift,
                m_robotContainer.m_collector),
            new InstantCommand(() -> m_robotContainer.m_lift.lowerLiftToCarry()),
            new InstantCommand(() -> m_robotContainer.m_collector.m_intakeState = IntakeState.OFF))
            .schedule();
        returnedCarryFromHigh = true;
        hasHighPoleApproached = false;

      }

    } else {

      /* Collector */
      if (!m_driver1_A.getState()) {
        hasGroundCollectionRun = false;
      }

      if (m_driver1_A.getState() || m_driver1_X.getState()) {
        hasShelfCarryRetractedRun = false;
        hasCarryRetractedRun = false;
      }

      /* Lift and Extension States */
      if (m_driver1_X.getState()) {

        SmartDashboard.putString("Lift Extension", "Shelf");
        m_robotContainer.m_shelfCollection.schedule();
        new InstantCommand(
            () -> m_robotContainer.m_collector.m_adjustableWheelMotorPower = CollectorConstants.k_wheelMotorSpeed)
            .schedule();
        m_robotContainer.m_collector.m_intakeState = IntakeState.IN;
        m_robotContainer.m_collector.m_collectingByCommand = true;

      } else if (m_driver1_A.getState() && !hasGroundCollectionRun) {
        SmartDashboard.putString("Lift Extension", "Ground");
        m_robotContainer.m_groundCollection.schedule();
        hasGroundCollectionRun = true;
        m_robotContainer.m_collector.m_intakeState = IntakeState.IN;
        m_robotContainer.m_collector.m_collectingByCommand = true;

      } else if ((!m_driver1_A.getState() && !m_driver1_X.getState())) {
        SmartDashboard.putString("Lift Extension", "Carry");

        if (m_robotContainer.m_lift.m_state == LiftPosition.SHELF_COLLECTION && !hasShelfCarryRetractedRun) {

          hasShelfCarryRetractedRun = true;
          new ShelfCarryRetractedCommandGroup(m_robotContainer.m_extension, m_robotContainer.m_lift,
              m_robotContainer.m_collector).schedule();
          m_robotContainer.m_collector.overLimit = false;
          new InstantCommand(() -> m_robotContainer.m_collector.m_intakeState = IntakeState.HOLD_IN).schedule();

        } else if (!hasCarryRetractedRun) {

          // m_robotContainer.m_collector.objectCollected = false;
          // m_robotContainer.m_collector.overLimit = false;

          hasCarryRetractedRun = true;
          if (m_robotContainer.m_lift.m_state == LiftPosition.SHELF_COLLECTION) {
            new ShelfCarryRetractedCommandGroup(m_robotContainer.m_extension, m_robotContainer.m_lift,
                m_robotContainer.m_collector).schedule();
          } else {
            m_robotContainer.m_carryRetracted.schedule();
          }

          new InstantCommand(
              () -> m_robotContainer.m_collector.m_adjustableWheelMotorPower = CollectorConstants.k_wheelMotorSpeed)
              .schedule();
          new InstantCommand(() -> m_robotContainer.m_collector.m_intakeState = IntakeState.HOLD_IN).schedule();

        }
      }

      if (m_robotContainer.m_lift.m_state == LiftPosition.CARRY) {
        m_robotContainer.m_collector.firstTIme = true;
      }

      // /* Intake Control */
      // if (m_robotContainer.m_driver2.getRawAxis(JoystickConstants.k_rightTrigger) >
      // 0.5) {
      // m_robotContainer.m_collector.m_intakeState = IntakeState.IN;
      // m_robotContainer.m_collector.m_collectingByCommand = false;
      // } else if (!m_robotContainer.m_collector.m_collectingByCommand) {
      // m_robotContainer.m_collector.m_intakeState = IntakeState.OFF;
      // }

    }

    m_robotContainer.m_driver1LeftBumper
        .toggleOnTrue(new InstantCommand(() -> m_robotContainer.m_collector.m_collectorState = CollectorState.OPEN));

    m_robotContainer.m_zeroGyro.whileTrue(new InstantCommand(() -> m_robotContainer.m_swerve.zeroGyro()));

    // TELEMETRY DATA
    // //////////////////////////////////////////////////////////////////////

    SmartDashboard.putBoolean("Driver 1 A Button State", m_driver1_A.getState());
    SmartDashboard.putString("ROBOT MODE", s_robotMode.toString());

    SmartDashboard.putBoolean("C - FIRST TIME", m_robotContainer.m_collector.firstTIme);

  }

  // code bombs are superior to any other kind of bomb
  // theres a code bomb in our code.

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }
}