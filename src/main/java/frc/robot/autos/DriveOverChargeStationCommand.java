package frc.robot.autos;

import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Swerve.SwerveConstants;
import java.util.List;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class DriveOverChargeStationCommand extends CommandBase {

    public static final class DriveOverChargeStationCommandAutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double k_maxSpeedMetersPerSecond = 3;
        public static final double k_maxAccelerationMetersPerSecondSquared = 3;
        public static final double k_maxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double k_maxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double k_pXController = 1;
        public static final double k_pYController = 1;
        public static final double k_pThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints k_thetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                k_maxAngularSpeedRadiansPerSecond, k_maxAngularSpeedRadiansPerSecondSquared);
    }

    Swerve m_swerve;
    boolean drivenOverChargeStation = false;
    boolean goingDown = false;
    TeleopSwerve m_teleopSwerve;
    
    public DriveOverChargeStationCommand(Swerve p_Swerve){
        this.m_swerve = p_Swerve;
        addRequirements(m_swerve);
    }

    @Override
    public void initialize(){
        m_teleopSwerve = new TeleopSwerve(
            m_swerve,
            () -> -0.3,
            () -> 0,
            () -> 0,
            () -> false
        );
    }

    @Override
    public void execute(){
   

        if(goingDown){
            if(m_swerve.getTilt() < -1){
                m_teleopSwerve.execute();
            } else {
                m_teleopSwerve = new TeleopSwerve(
                    m_swerve,
                    () -> 0,
                    () -> 0,
                    () -> 0,
                    () -> false
                );
                m_teleopSwerve.execute();
                drivenOverChargeStation = true;
            }
        } else if(m_swerve.getTilt() > -10){
            m_teleopSwerve.execute();
        } else if(!goingDown){
            goingDown = true;
        }
    }

    @Override
    public boolean isFinished(){
        if(drivenOverChargeStation){
            goingDown = false;
            drivenOverChargeStation = false;
            m_teleopSwerve = new TeleopSwerve(
                m_swerve,
                () -> -0.3,
                () -> 0,
                () -> 0,
                () -> false);
            return true;
        } else {
            return false;
        }
    }
}