package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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

    private static final String profileDefault = "Default";
    private static final String highSpeed = "High Speed";
    private static final String highAccuracy = "High Accuracy";
    private static final String longRange = "Long Range";
    private String m_profileSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();
    
    private Rev2mDistanceSensor distSens; //look for why is this not importing correctly

@Override
public void initialize() {
    m_collectorState = CollectorState.OFF;
    m_chooser.setDefaultOption("Default", profileDefault);
    m_chooser.addOption("High Speed", highSpeed);
    m_chooser.addOption("High Accuracy", highAccuracy);
    m_chooser.addOption("Long Range", longRange);
    SmartDashboard.putData("Profile", m_chooser);
    distSens = new Rev2mDistanceSensor(Port.kOnboard);
    distSens.setAutomaticMode(true);
    distSens.setEnabled(true);
}

@Override
public void periodic() {
    setCollectorState();

    switch (m_profileSelected) {
        case highSpeed:
          distSens.setRangeProfile(RangeProfile.kHighSpeed);
          break;
        case highAccuracy:
          distSens.setRangeProfile(RangeProfile.kHighAccuracy);
          break;
        case longRange:
          distSens.setRangeProfile(RangeProfile.kLongRange);
          break;
        default:
          distSens.setRangeProfile(RangeProfile.kDefault);
          break;
      }

    boolean isValid = distSens.isRangeValid();
    SmartDashboard.putBoolean("Valid", isValid);
    if(isValid) {
        SmartDashboard.putNumber("Range", distSens.getRange());
        SmartDashboard.putNumber("Timestamp", distSens.getTimestamp());
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
    public void disablePeriodic() {
        m_collectorState = CollectorState.OFF;
        distSens.setEnabled(false); //this might cause a problem, not sure until tested 
    }


    @Override
    public void enablePeriodic() {
        
    }

} 