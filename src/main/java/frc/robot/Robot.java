// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.cscore.CameraServerJNI.TelemetryKind;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer.JoystickConstants;
//import frc.robot.autos.AutoCenter;
import frc.robot.commandGroups.CarryRetractedCommandGroup;
import frc.robot.commandGroups.DepositSequenceCommandGroup;
import frc.robot.commandGroups.DepositSequenceHighPoleCommandGroup;
import frc.robot.commandGroups.GroundCollectionCommandGroup;
import frc.robot.commandGroups.GroundCollectionSequenceCommandGroup;
import frc.robot.commandGroups.HighPoleApproachCommandGroup;
import frc.robot.commandGroups.LowPoleApproachCommandGroup;
import frc.robot.commandGroups.ShelfCarryRetractedCommandGroup;
import frc.robot.commandGroups.ShelfCollectionApproachCommandGroup;
import frc.robot.commands.GreenMonkDrive;
import frc.robot.commands.extensionCommands.ExtensionDepositSequenceCommand;
import frc.robot.subsystems.NewCollector;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.NewCollector.CollectorConstants;
import frc.robot.subsystems.NewCollector.CollectorState;
import frc.robot.subsystems.NewCollector.IntakeState;
import frc.robot.subsystems.Extension.TelescopePosition;
import frc.robot.subsystems.Lift.HookState;
import frc.robot.subsystems.Lift.LiftPosition;
import frc.robot.utilities.BrainSTEMSubsystem;
import frc.robot.utilities.LimelightHelpers;
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

  public enum DepositLocation {
    HIGH, 
    LOW,
    GROUND
  }  

  public static RobotMode s_robotMode = RobotMode.COLLECTING;

  public static DepositLocation s_depositLocation = DepositLocation.LOW;

  private ToggleButton m_driver1_A = new ToggleButton();
  private ToggleButton m_driver1_X = new ToggleButton();
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

  public static double limelightCurrent;
  //private boolean hasLimelightRun = false;


  

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
    brainSTEMSubsystemsWithoutSwerve = m_robotContainer.getBrainSTEMSubsystems();
    brainSTEMSubsystemsWithoutSwerve.remove(m_robotContainer.m_swerve);
    
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
     //m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    ArrayList<BrainSTEMSubsystem> brainSTEMSubsystems = m_robotContainer.getBrainSTEMSubsystems();
    for(BrainSTEMSubsystem isubsystem: brainSTEMSubsystems){
      isubsystem.initialize();
    }
    //AutoCenter.sideAuto = true;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if((m_robotContainer.m_swerve.getTilt() > 5) || (m_robotContainer.m_swerve.getTilt() < -5)) {
      //AutoCenter.sideAuto = false;
    }
  }

  @Override
  public void teleopInit() {
    //hasLimelightRun = false;
    
    
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

    m_robotContainer.m_lift.m_state = LiftPosition.CARRY;
  }

  private void setToggleButtons() {

    // m_driver1_A.update(m_robotContainer.m_driver1AButton.getAsBoolean() && !);
    // m_driver1_X.update(m_robotContainer.m_driver1XButton.getAsBoolean() && !m_robotContainer.m_driver1AButton.getAsBoolean());
    m_driver1_A.update(m_robotContainer.m_driver1AButton.getAsBoolean());
    m_driver1_X.update(m_robotContainer.m_driver1XButton.getAsBoolean());
    m_driver1_B.update(m_robotContainer.m_driver1BButton.getAsBoolean());
    if (m_robotContainer.m_driver1AButton.getAsBoolean()) {
      m_driver1_X.setState(false);
    } else if (m_robotContainer.m_driver1XButton.getAsBoolean()) {
      m_driver1_A.setState(false);
    } 
  }

  private void setRobotState() {
    if (m_robotContainer.m_driver2AButton.getAsBoolean()) {
      m_driver1_X.setState(false);
      m_driver1_A.setState(false);
      //new InstantCommand(() -> m_robotContainer.m_collector.m_adjustableClawMotorPower = CollectorConstants.k_clawMotorHoldingSpeed).schedule();
      //m_robotContainer.m_collector.m_collectorState = CollectorState.CLOSED;
      s_robotMode = RobotMode.COLLECTING;
    } else if (m_robotContainer.m_driver2BButton.getAsBoolean()) {
      m_driver1_X.setState(false);
      m_driver1_A.setState(false);
      s_robotMode = RobotMode.DEPOSITING;
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Limelight Light Current ", limelightCurrent);

    limelightCurrent = LimelightHelpers.getTX("limelight");

    //CODE TO TEST OUT HOOK UP AND DOWN POSITIONS
    // if(m_robotContainer.m_driver2YButton.getAsBoolean()) {
    //   new InstantCommand(() -> m_robotContainer.m_lift.m_hookState = HookState.DOWN).schedule();
    // }

    // if(m_robotContainer.m_driver2XButton.getAsBoolean()) {
    //   new InstantCommand(() -> m_robotContainer.m_lift.m_hookState = HookState.UP).schedule();
    // }

    //CODE TO SET COAST OR BRAKE MODE ON DRIVETRAIN
    // if(m_robotContainer.m_driver2YButton.getAsBoolean()) {
    //   m_robotContainer.m_swerve.m_adjustableDriveNeutralMode = NeutralMode.Brake;
    //   m_robotContainer.m_swerve.m_adjustableAngleNeutralMode = NeutralMode.Brake;
    // } else if (m_robotContainer.m_driver2XButton.getAsBoolean()) {
    //   m_robotContainer.m_swerve.m_adjustableDriveNeutralMode = NeutralMode.Coast;
    //   m_robotContainer.m_swerve.m_adjustableAngleNeutralMode = NeutralMode.Coast;
    // }
    // if(!hasLimelightRun) {
    //   m_robotContainer.m_limelightCommand.schedule();
    //   hasLimelightRun = true;
    // }
    

    if(m_robotContainer.m_driver1BButton.getAsBoolean()) {
      m_robotContainer.monkDrive.schedule();
      hasMonkDriveCanceled = false;
    } else if(!hasMonkDriveCanceled){
      m_robotContainer.monkDrive.cancel();
      hasMonkDriveCanceled = true;
    } 

    // if(m_robotContainer.m_driver2YButton.getAsBoolean()) {
    //   m_robotContainer.limelightMonkDrive.schedule();
    //   hasLimelightMonkDriveCanceled = false;
    // } else if(!hasLimelightMonkDriveCanceled){
    //   m_robotContainer.limelightMonkDrive.cancel();
    //   hasLimelightMonkDriveCanceled = true;
    //}

  

    if(m_robotContainer.m_driver1.getPOV() == 90) {
      SmartDashboard.putString("D1 POV","Right");
      m_robotContainer.rightGreenMonkDrive.schedule();
    } else if (m_robotContainer.m_driver1.getPOV() == 270) {
      SmartDashboard.putString("D1 POV","Left");
      m_robotContainer.leftGreenMonkDrive.schedule();
    } else {
      m_robotContainer.rightGreenMonkDrive.cancel();
      m_robotContainer.leftGreenMonkDrive.cancel();
    }

    m_driver1_BackButton.update(m_robotContainer.m_driver1.getRawButton(JoystickConstants.k_backButton));
    setRobotState();
    setToggleButtons();
    if(m_driver1_BackButton.getState()) {
      for(BrainSTEMSubsystem i_subsystem : brainSTEMSubsystemsWithoutSwerve){
        i_subsystem.disablePeriodic();
      }
      m_robotContainer.m_lift.turnOffLiftMotors();
      m_robotContainer.m_extension.turnOffExtensionMotor();
      //m_robotContainer.m_collector.killCollectorMotors();
    } else {
      for(BrainSTEMSubsystem i_subsystem : brainSTEMSubsystemsWithoutSwerve){
        i_subsystem.enablePeriodic();
      }
    }

    SmartDashboard.putBoolean("Driver 1 A Button State", m_driver1_A.getState());

    SmartDashboard.putString("Robot Mode", s_robotMode.toString());

    /* Driver Buttons */

    if(m_robotContainer.m_driver1.getRawAxis(JoystickConstants.k_rightTrigger) < 0.7) {
      hasDepositingRun = false;
    }

    if(s_robotMode == RobotMode.DEPOSITING) {
      if(m_robotContainer.m_driver1.getRawAxis(JoystickConstants.k_rightTrigger) > 0.7 && !hasDepositingRun) {
        hasDepositingRun = true;
        if (s_depositLocation == DepositLocation.HIGH) {
          //new DepositSequenceHighPoleCommandGroup(m_robotContainer.m_lift, m_robotContainer.m_extension, m_robotContainer.m_collector).schedule();
        } else {
          //new DepositSequenceCommandGroup(m_robotContainer.m_lift, m_robotContainer.m_extension, m_robotContainer.m_collector).schedule(); 
        }
      }
      if (m_robotContainer.m_driver1XButton.getAsBoolean() && (m_robotContainer.m_lift.m_state != LiftPosition.HIGH_POLE_TILT) && (m_robotContainer.m_lift.m_state != LiftPosition.HIGH_POLE)) { // && (m_robotContainer.m_lift.m_state != LiftPosition.HIGH_POLE_TILT) && (m_robotContainer.m_lift.m_state != LiftPosition.HIGH_POLE)
        s_depositLocation = DepositLocation.LOW;
        //new LowPoleApproachCommandGroup(m_robotContainer.m_extension, m_robotContainer.m_lift, m_robotContainer.m_collector).schedule();
      }
      if (m_robotContainer.m_driver1YButton.getAsBoolean() && (m_robotContainer.m_lift.m_state != LiftPosition.LOW_POLE)) { //&& (m_robotContainer.m_lift.m_state != LiftPosition.LOW_POLE)
        s_depositLocation = DepositLocation.HIGH;
        //new HighPoleApproachCommandGroup(m_robotContainer.m_extension, m_robotContainer.m_lift, m_robotContainer.m_collector).schedule();
      } 
      
      
    } else {
      
      /* Collector */
      if(!m_driver1_A.getState()) {
        hasGroundCollectionRun = false;
      }

      if(m_driver1_A.getState() || m_driver1_X.getState()) {
        hasShelfCarryRetractedRun = false;
        hasCarryRetractedRun = false;
      }

      /* Lift and Extension States */
      if (m_driver1_X.getState()) {
        SmartDashboard.putString("Lift Extension", "Shelf");
        m_robotContainer.m_shelfCollection.schedule();        //new InstantCommand(() -> m_robotContainer.m_collector.m_intakeState = IntakeState.IN).schedule();
      } else if (m_driver1_A.getState() && !hasGroundCollectionRun) {
        SmartDashboard.putString("Lift Extension", "Ground");
        //m_robotContainer.m_groundCollection.schedule();
        hasGroundCollectionRun = true;
      } else if ((!m_driver1_A.getState() && !m_driver1_X.getState())) {
        SmartDashboard.putString("Lift Extension", "Carry");
        if(m_robotContainer.m_lift.m_state == LiftPosition.SHELF_COLLECTION && !hasShelfCarryRetractedRun) {
          hasShelfCarryRetractedRun = true;
          //new ShelfCarryRetractedCommandGroup(m_robotContainer.m_extension, m_robotContainer.m_lift, m_robotContainer.m_collector).schedule();
        } else if (!hasCarryRetractedRun){
          hasCarryRetractedRun = true;
          m_robotContainer.m_carryRetracted.schedule();
        }
      }

      /* Intake Control */
      if (m_robotContainer.m_driver1.getRawAxis(JoystickConstants.k_rightTrigger) > 0.5) {
        //m_robotContainer.m_collector.m_adjustableWheelMotorPower = 0.3;
        //m_robotContainer.m_collector.m_intakeState = IntakeState.IN;
      } else {
        //m_robotContainer.m_collector.m_adjustableWheelMotorPower = CollectorConstants.k_wheelMotorSpeed;
        //m_robotContainer.m_collector.m_intakeState = IntakeState.OFF;
      }


    }

    //m_robotContainer.m_driver1LeftBumper.toggleOnTrue(new InstantCommand(() -> m_robotContainer.m_collector.m_collectorState = CollectorState.OPEN));
    
    

    m_robotContainer.m_zeroGyro.whileTrue(new InstantCommand(() -> m_robotContainer.m_swerve.zeroGyro()));

  }

  //code bombs are superior to any other kind of bomb

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}