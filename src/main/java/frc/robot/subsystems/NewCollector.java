package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.BrainSTEMSubsystem;


public class NewCollector extends SubsystemBase implements BrainSTEMSubsystem{


    public static final class CollectorConstants {

        // Grabber Wheel Motors ///////////////////////////////////////
        public static int k_rightMotorMotorID = 19;
        public static int k_leftMotorMotorID = 22;
        public static double k_holdingSpeed = -0.2;
        public static double k_colletSpeed = -0.2;
        public static double k_depositSpeed = 0.2;
        public static boolean k_rightMotorInverted = false;
        public static boolean k_leftMotorInverted = false;

        // Ultrasonic Sensor /////////////////////////////////////////
        public static int k_ultrasonicPort1 = 1;
        public static int k_ultrasonicPort2 = 2;

        public static double k_cargoInPositionIN = 2; // FIXME
        public static double k_cargoInPositionMM = 10; // FIXME
    }

    
    private CANSparkMax m_rightMotor;
    private CANSparkMax m_leftMotor;
    private Ultrasonic m_collectorSensor;

    


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

    public enum CargoState {
        COLLECTED, 
        NULL
    }

    public CollectorState m_collectorState = CollectorState.OFF;
    public IntakeState m_intakeState = IntakeState.IN;
    public CargoState m_cargoState = CargoState.COLLECTED;



    public NewCollector() {

        m_leftMotor = new CANSparkMax(CollectorConstants.k_rightMotorMotorID, MotorType.kBrushless);
        m_rightMotor = new CANSparkMax(CollectorConstants.k_rightMotorMotorID, MotorType.kBrushless);

        m_leftMotor.setInverted(CollectorConstants.k_leftMotorInverted);
        m_rightMotor.setInverted(CollectorConstants.k_rightMotorInverted);

        

        m_collectorSensor = new Ultrasonic(CollectorConstants.k_ultrasonicPort1, CollectorConstants.k_ultrasonicPort2);
        
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

    public double cargoPositionMM() {
        return m_collectorSensor.getRangeMM();
    }

    public double cargoPositionIN() {
        return m_collectorSensor.getRangeInches();
    }

    public boolean isCargoCollectedMM()  {
        return (cargoPositionMM() < CollectorConstants.k_cargoInPositionMM);
    }

    public boolean isCargoCollectedIN()  {
        return (cargoPositionIN() < CollectorConstants.k_cargoInPositionIN);
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

    public void setCargoState() {
        switch(m_cargoState) {
            case COLLECTED: 
                m_cargoState = CargoState.COLLECTED;
            break;
            case NULL:
                m_cargoState = CargoState.NULL;
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
        setIntakeState();
        setCargoState();

        SmartDashboard.putString("Collector Data", "--------------------------");
        SmartDashboard.putNumber("Cargo Position MM", cargoPositionMM());
        SmartDashboard.putNumber("Cargo Position IN", cargoPositionIN());

    
    }


    @Override
    public void disablePeriodic() {
        m_collectorState = CollectorState.OFF;
    }


    @Override
    public void enablePeriodic() {
        
    }

} 