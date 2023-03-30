package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.BrainSTEMSubsystem;

public class NewCollector extends SubsystemBase implements BrainSTEMSubsystem{


    public static final class CollectorConstants {
        public static int k_rightMotorMotorID = 19;
        public static int k_leftMotorMotorID = 22;
        public static double k_holdingSpeed = -0.2;
        public static double k_colletSpeed = -0.2;
        public static double k_depositSpeed = 0.2;
        public static boolean k_rightMotorInverted = false;
        public static boolean k_leftMotorInverted = false;

    }

    private CANSparkMax m_rightMotor;
    private CANSparkMax m_leftMotor;


    public enum CollectorState {
        COLLECT,
        DEPOSIT,
        HOLD,
        OFF
    }

    public enum IntakeState {
        IN, 
        OUT
    }

    public CollectorState m_collectorState = CollectorState.OFF;
    public IntakeState m_intakeState = IntakeState.IN;



    public NewCollector() {

        m_leftMotor = new CANSparkMax(CollectorConstants.k_rightMotorMotorID, MotorType.kBrushless);
        m_rightMotor = new CANSparkMax(CollectorConstants.k_rightMotorMotorID, MotorType.kBrushless);

        m_leftMotor.setInverted(CollectorConstants.k_leftMotorInverted);
        m_rightMotor.setInverted(CollectorConstants.k_rightMotorInverted);


        
    }


    public void collectorCollect() {
        m_leftMotor.set(CollectorConstants.k_colletSpeed);
        m_rightMotor.follow(m_leftMotor);
    }

    public void collectorHold() {
        m_leftMotor.set(CollectorConstants.k_holdingSpeed);
        m_rightMotor.follow(m_leftMotor);
    }

    public void collectorDeposit() {
        m_leftMotor.set(CollectorConstants.k_depositSpeed);
        m_rightMotor.follow(m_leftMotor);
    }

    public void collectorStop() {
        m_leftMotor.setIdleMode(IdleMode.kBrake);
        m_leftMotor.set(0);
        m_rightMotor.follow(m_leftMotor);
    }

    public void setIntakeState() {
        switch(m_intakeState) {
            case IN:
                m_collectorState = CollectorState.COLLECT;
            break;
            case OUT:
                m_collectorState = CollectorState.DEPOSIT;
            break;
        }
    }


    public void setCollectorState() {
        switch(m_collectorState) {
            case COLLECT:
                collectorCollect();
            break;
            case HOLD:
                collectorCollect();
            break;
            case DEPOSIT:
                collectorDeposit();
            break;
            case OFF:
                collectorStop();
            break;

        }
    }

    

    @Override
    public void initialize() {
        m_collectorState = CollectorState.OFF;
    }

    @Override
    public void periodic() {
        setCollectorState();
        
    }

    @Override
    public void disablePeriodic() {
        m_collectorState = CollectorState.OFF;
    }


    @Override
    public void enablePeriodic() {
        
    }

} 