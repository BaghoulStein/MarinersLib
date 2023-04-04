package frc.MarinersLib.motorUtil;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.PIDController;


public class SparkMaxController extends CANSparkMax {
    private SparkMaxPIDController m_PIDController;
    private RelativeEncoder m_Encoder;

    public SparkMaxController(int deviceNumber, PIDFGains motorPID) {
        super(deviceNumber, MotorType.kBrushed);

        m_Encoder = getEncoder();
        m_PIDController = getPIDController();

        m_PIDController.setP(motorPID.getP());
        m_PIDController.setI(motorPID.getI());
        m_PIDController.setD(motorPID.getD());
        m_PIDController.setIZone(motorPID.getIZone());
    }

    public SparkMaxController(int deviceNumber) {
        this(deviceNumber, new PIDFGains(0, 0, 0));
    }
}

