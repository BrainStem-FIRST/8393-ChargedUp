// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer.JoystickConstants;
import frc.robot.commandGroups.CarryRetractedCommandGroup;
import frc.robot.commandGroups.DepositSequenceCommandGroup;
import frc.robot.commandGroups.GroundCollectionCommandGroup;
import frc.robot.commandGroups.GroundCollectionSequenceCommandGroup;
import frc.robot.commandGroups.HighPoleApproachCommandGroup;
import frc.robot.commandGroups.LowPoleApproachCommandGroup;
import frc.robot.commandGroups.ShelfCollectionApproachCommandGroup;
import frc.robot.commands.extensionCommands.ExtensionDepositSequenceCommand;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Collector.CollectorState;
import frc.robot.subsystems.Collector.IntakeState;
import frc.robot.subsystems.Extension.TelescopePosition;
import frc.robot.subsystems.Lift.LiftPosition;
import frc.robot.utilities.BrainSTEMSubsystem;
import frc.robot.utilities.ToggleButton;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public enum RobotMode {
    COLLECTING, 
    DEPOSITING
    }

  public static RobotMode s_robotMode = RobotMode.COLLECTING;

  private ToggleButton m_driver1_A = new ToggleButton();
  private ToggleButton m_driver1_X = new ToggleButton();

  private ToggleButton m_driver2_A = new ToggleButton();
  private ToggleButton m_driver2_B = new ToggleButton();


  public static CTREConfigs ctreConfigs;

  private Command m_autonomousCommand;

  public static boolean depositSequenceContinue = true;

  private RobotContainer m_robotContainer;

  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    ctreConfigs = new CTREConfigs();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    ArrayList<BrainSTEMSubsystem> brainSTEMSubsystems = m_robotContainer.getBrainSTEMSubsystems();
    for(BrainSTEMSubsystem isubsystem: brainSTEMSubsystems){
      isubsystem.disablePeriodic();
    }
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
     m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    ArrayList<BrainSTEMSubsystem> brainSTEMSubsystems = m_robotContainer.getBrainSTEMSubsystems();
    for(BrainSTEMSubsystem isubsystem: brainSTEMSubsystems){
      isubsystem.initialize();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    ArrayList<BrainSTEMSubsystem> brainSTEMSubsystems = m_robotContainer.getBrainSTEMSubsystems();
    for(BrainSTEMSubsystem isubsystem: brainSTEMSubsystems){
      isubsystem.initialize();
    }
  }

  private void setToggleButtons() {
    if (m_robotContainer.m_driver1AButton.getAsBoolean()) {
      m_driver1_A.update(true);
      m_driver1_X.setState(false);
    } else if (m_robotContainer.m_driver1XButton.getAsBoolean()) {
      m_driver1_X.update(true);
      m_driver1_A.setState(false);
    }
  }

  private void setRobotState() {
    if (m_robotContainer.m_driver2AButton.getAsBoolean()) {
      m_robotContainer.m_collector.m_collectorState = CollectorState.CLOSED;
    } else if (m_robotContainer.m_driver2BButton.getAsBoolean()) {
      s_robotMode = RobotMode.DEPOSITING;
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    setRobotState();
    setToggleButtons();


    // if(s_robotMode == RobotMode.COLLECTING) {

    //   toggleCollectionButton.update(m_robotContainer.m_driver1.getRawButton(JoystickConstants.k_aButton));
    //   //SmartDashboard.putBoolean("collection button state", toggleCollectionButton.getState());
    //   if(toggleCollectionButton.getState()) {
    //     new ShelfCollectionApproachCommandGroup(m_robotContainer.m_extension, m_robotContainer.m_lift).schedule();
    //     //m_robotContainer.m_lowPoleApproach.schedule();
    //   } else {
    //     new CarryRetractedCommandGroup(m_robotContainer.m_extension, m_robotContainer.m_lift).schedule();
    //   }
    // } 

    // if((m_robotContainer.m_driver1.getRawAxis(JoystickConstants.k_rightTrigger) > 0.5)) {
    //   new DepositSequenceCommandGroup(m_robotContainer.m_lift, m_robotContainer.m_extension, m_robotContainer.m_collector).schedule();
    // } else if ((m_robotContainer.m_driver1.getRawAxis(JoystickConstants.k_leftTrigger) > 0.5)) {
    //   new GroundCollectionCommandGroup(m_robotContainer.m_extension, m_robotContainer.m_lift, m_robotContainer.m_collector).schedule();
    // }


    SmartDashboard.putString("Robot Mode", s_robotMode.toString());
    /* Driver Buttons */
    if(s_robotMode == RobotMode.DEPOSITING) {
      if(m_robotContainer.m_driver1.getRawAxis(JoystickConstants.k_rightTrigger) > 0.9) {
        new DepositSequenceCommandGroup(m_robotContainer.m_lift, m_robotContainer.m_extension, m_robotContainer.m_collector).schedule();
      }
      if (m_robotContainer.m_driver1XButton.getAsBoolean()) {
        new LowPoleApproachCommandGroup(m_robotContainer.m_extension, m_robotContainer.m_lift).schedule();
      }
      if (m_robotContainer.m_driver1YButton.getAsBoolean()) {
        new HighPoleApproachCommandGroup(m_robotContainer.m_extension, m_robotContainer.m_lift).schedule();
      }
      
      //m_driver1AButton.toggleOnTrue(new InstantCommand(() -> m_carryRetracted.schedule()));
      // m_driver1XButton.toggleOnTrue(new InstantCommand(() -> m_lowPoleApproach.schedule()));
    } else {

      /* Collector */

      /* Lift and Extension States */
      if (m_driver1_X.getState()) {
        SmartDashboard.putString("Lift Extension", "Shelf");
        m_robotContainer.m_shelfCollection.schedule();
        new InstantCommand(() -> m_robotContainer.m_collector.m_intakeState = IntakeState.IN).schedule();
      } else if (m_driver1_A.getState()) {
        SmartDashboard.putString("Lift Extension", "Ground");
        m_robotContainer.m_groundCollection.schedule();
      } else {
        SmartDashboard.putString("Lift Extension", "Carry");
        m_robotContainer.m_carryRetracted.schedule();
      }

      /* Intake Control */
      if (m_robotContainer.m_driver1.getRawAxis(JoystickConstants.k_rightTrigger) > 0.5) {
        m_robotContainer.m_collector.m_intakeState = IntakeState.IN;
      } else {
        m_robotContainer.m_collector.m_intakeState = IntakeState.OFF;
      }

    }

    m_robotContainer.m_driver1LeftBumper.toggleOnTrue(new InstantCommand(() -> m_robotContainer.m_collector.m_collectorState = CollectorState.OPEN));
    
    //m_driver1RightBumper.toggleOnTrue(new InstantCommand(() -> m_collector.m_intakeState = IntakeState.OFF));


    
    
    
    //m_driver2XButton.toggleOnTrue(new InstantCommand(() -> m_extension.scheduleLowPole()));
    //m_driver2YButton.toggleOnTrue(new InstantCommand(() -> m_extension.scheduleHighPole()));


    m_robotContainer.m_zeroGyro.whileTrue(new InstantCommand(() -> m_robotContainer.m_swerve.zeroGyro()));

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
