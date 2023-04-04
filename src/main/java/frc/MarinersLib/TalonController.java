package frc.MarinersLib;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/** Universal replacement for talon controllers */
public class TalonController extends BaseTalon {
    public static enum MotorType {
        FALCON500,
        SRX
    }

    private PIDFGains m_MotorPID;

    private double m_GearRatio;
    private double m_EncoderTicks2Rotation;

    private boolean m_IsLocked;
    private double m_LockedPosition;

    /**
     * Extension to the BaseTalon class. Created to be a universal replacement to
     * the {@link com.ctre.phoenix.motorcontrol.can.TalonFX} and
     * {@link com.ctre.phoenix.motorcontrol.can.TalonSRX} classes.
     * 
     * 
     * @param deviceNumber Motor ID on the CANBus chain.
     * @param motorType Type of the motor (Falcon or SRX). Impacts the encoder readings.
     * @param motorPID PIDFGains of the motor.
     * @param gearRatio Ratio between the motor -> end goal. Should be formatted as 1/val.
     * @param isInverted Whether the motor is inverted.
     */
    public TalonController(int deviceNumber, MotorType motorType, PIDFGains motorPID, double gearRatio,
            boolean isInverted) {
        super(deviceNumber, "");

        configFactoryDefault();
        configSelectedFeedbackSensor(
                motorType == MotorType.FALCON500
                        ? FeedbackDevice.IntegratedSensor
                        : FeedbackDevice.QuadEncoder,
                0, 0);
        configPeakOutputForward(1.0);
        configPeakOutputReverse(-1.0);
        setNeutralMode(NeutralMode.Brake);
        setInverted(isInverted);

        this.m_EncoderTicks2Rotation = motorType == MotorType.FALCON500
                ? 2048.0
                : 1024.0;
        this.m_GearRatio = gearRatio;
        this.m_MotorPID = motorPID;
        this.m_IsLocked = false;
        this.m_LockedPosition = 0.0;

        config_kP(0, m_MotorPID.getP());
        config_kI(0, m_MotorPID.getI());
        config_kD(0, m_MotorPID.getD());
        config_kF(0, m_MotorPID.getF());
        configAllowableClosedloopError(0, m_MotorPID.getTolerance());
        setIntegralAccumulator(0);
        setSelectedSensorPosition(0);
    }

    public TalonController(int deviceNumber, MotorType motorType) {
        this(deviceNumber, motorType, new PIDFGains(0, 0, 0), 1.0, false);
    }

    public TalonController(int deviceNumber, MotorType motorType, double gearRatio) {
        this(deviceNumber, motorType, new PIDFGains(0, 0, 0), gearRatio, false);
    }

    public TalonController(int deviceNumber, MotorType motorType, PIDFGains PIDFgains) {
        this(deviceNumber, motorType, PIDFgains, 1.0, false);
    }

    public TalonController(int deviceNumber, MotorType motorType, boolean isInverted) {
        this(deviceNumber, motorType, new PIDFGains(0, 0, 0), 1.0, isInverted);
    }

    /**
     * Resets the motor values such as encoder readings and integral accumulator
     */
    public void resetMotor() {
        set(ControlMode.PercentOutput, 0);
        m_IsLocked = false;
        m_LockedPosition = 0;

        setSelectedSensorPosition(0);
        setIntegralAccumulator(0);
    }

    /**
     * sets the %output of the motor
     * @param PercentOutput setter value
     */
    public void set(double PercentOutput) {
        set(ControlMode.PercentOutput, PercentOutput);
    }

    /**
     * @return current velcoity of the motor in RPS
     */
    public double getVelocity() {
        return getSelectedSensorVelocity() * 10 * m_EncoderTicks2Rotation * m_GearRatio; // getSelectedSensorVelocity
                                                                                         // returns in 100ms values so
                                                                                         // needs to be multiplied to
                                                                                         // reach second
    }

    /**
     * @return current motor position in rotations
     */
    public double getRotations() {
        return getSelectedSensorPosition() / m_EncoderTicks2Rotation * m_GearRatio;
    }

    public void setVelocity(double rps) {
        if (rps == 0) {
            lockPosition();
            return;
        }
        m_IsLocked = false;

        set(ControlMode.Velocity, rps / m_GearRatio * m_EncoderTicks2Rotation);
    }

    public void setPosition(double rotations) {
        set(ControlMode.Position, rotations / m_GearRatio * m_EncoderTicks2Rotation);
    }

    public void lockPosition() {
        if (!m_IsLocked) {
            m_IsLocked = true;
            m_LockedPosition = getRotations();
        }

        setPosition(m_LockedPosition);
    }

    /**
     * Creates/Gets a shuffleboard tab specifically for the mtoor
     * where it displays all relavent information about the motors state.
     */
    public void debugMotor() {
        ShuffleboardTab tab = Shuffleboard.getTab(String.format("TalonController ID: %d", getDeviceID()));
        tab.addDouble("Motor Position", () -> getRotations()).withWidget(BuiltInWidgets.kGraph);
        tab.addDouble("Motor Velocity", () -> getVelocity()).withWidget(BuiltInWidgets.kGraph);
        tab.addDouble("Motor Error", () -> getClosedLoopError()).withWidget(BuiltInWidgets.kGraph);
        tab.addDouble("Motor Target", () -> getClosedLoopTarget());
        tab.addBoolean("Motor Locked", () -> m_IsLocked);
        tab.addDouble("Motor Locked Position", () -> m_LockedPosition);
        tab.add(m_MotorPID);
    }

}
