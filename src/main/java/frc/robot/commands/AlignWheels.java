package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.SwerveModule;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Swerve.SwerveConstants;
import frc.robot.subsystems.Swerve.SwerveConstants.Mod0Constants;
import frc.robot.subsystems.Swerve.SwerveConstants.Mod1Constants;
import frc.robot.subsystems.Swerve.SwerveConstants.Mod2Constants;
import frc.robot.subsystems.Swerve.SwerveConstants.Mod3Constants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AlignWheels extends CommandBase {
        private Swerve s_Swerve;
        private Timer m_timer;

        public AlignWheels(Swerve s_Swerve) {
                this.s_Swerve = s_Swerve;
                addRequirements(s_Swerve);
        }

        @Override
        public void initialize() {
                m_timer.reset();
                m_timer.start();
        }

        @Override
        public void execute() {
                for(SwerveModule mod : s_Swerve.mSwerveMods){
                        Rotation2d angleOffset = new Rotation2d(0);
                        switch(mod.moduleNumber) {
                                case 0: {
                                        angleOffset = Mod0Constants.k_angleOffset;
                                        break;
                                }
                                case 1: {
                                        angleOffset = Mod1Constants.k_angleOffset;
                                        break;
                                }
                                case 2: {
                                        angleOffset = Mod2Constants.k_angleOffset;
                                        break;
                                }
                                case 3: {
                                        angleOffset = Mod3Constants.k_angleOffset;
                                        break;
                                }
                        }
                        mod.setAngle(new SwerveModuleState(1, angleOffset)); //s_Swerve.swerveModuleStates[mod.moduleNumber]
                }

        }

        @Override
        public boolean isFinished() {
                if(m_timer.get() > 1) {
                        for (SwerveModule mod : s_Swerve.mSwerveMods) {
                                mod.setAngleEncodersToZero();
                        }
                        return true;
                } else {
                        return false;
                }
        }
}