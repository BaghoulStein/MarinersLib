package frc.MarinersLib.motorUtil;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;


public class SparkMaxController extends CANSparkMax {
    private SparkMaxPIDController m_PIDController;
    private RelativeEncoder m_Encoder;

    public SparkMaxController(int deviceNumber, PIDFGains gains, double positionFactor, double velocityFactor,
            boolean isInverted) {
        super(deviceNumber, MotorType.kBrushed);

        m_Encoder = getEncoder();
        m_PIDController = getPIDController();

        setPIDF(gains);
        setInverted(isInverted);
        setConversionFactors(positionFactor, velocityFactor);
        setSmartCurrentLimit(40);
        enableVoltageCompensation(12);
        setIdleMode(IdleMode.kBrake);
        setOpenLoopRampRate(0.01);
        setClosedLoopRampRate(0.01);
    }

    public SparkMaxController(int deviceNumber, PIDFGains gains, boolean isInverted) {
        this(deviceNumber, gains, 1.0, 1.0, isInverted);
    }

    public SparkMaxController(int deviceNumber, double positionFactor, double velocityFactor, boolean isInverted) {
        this(deviceNumber, new PIDFGains(0, 0, 0), positionFactor, velocityFactor, isInverted);
    }

    public SparkMaxController(int deviceNumber, PIDFGains gains, double positionFactor, double velocityFactor) {
        this(deviceNumber, gains, positionFactor, velocityFactor, false);
    }

    public void setConversionFactors(double positionFactor, double velocityFactor) {
        m_Encoder.setPositionConversionFactor(positionFactor);
        m_Encoder.setVelocityConversionFactor(velocityFactor);
    }

    public void setPIDF(PIDFGains gains) {
        m_PIDController.setP(gains.getP());
        m_PIDController.setI(gains.getI());
        m_PIDController.setD(gains.getD());
        m_PIDController.setFF(gains.getF());
        m_PIDController.setIZone(gains.getIZone());
    }

    public void setReference(double value, ControlType controlType) {
        m_PIDController.setReference(value, controlType);
    }

    public void setPosition(double position) {
        m_Encoder.setPosition(position);
    }

    public double getVelocity() {
        return m_Encoder.getVelocity();
    }

    public double getPosition() {
        return m_Encoder.getPosition();
    }
}

