package frc.MarinersLib.motorUtil;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class PIDFGains implements Sendable{
    private double _kP, _kI, _kD, _kF, _tolerance, _iZone;

    public PIDFGains(double kP, double kI, double kD, double kF, double tolerance, double iZone) {
        this._kP = kP;
        this._kI = kI;
        this._kD = kD;
        this._kF = kF;
        this._tolerance = tolerance;
        this._iZone = iZone;
    }

    public PIDFGains(double kP, double kI, double kD) {
        this._kP = kP;
        this._kI = kI;
        this._kD = kD;
        this._kF = 0;
        this._tolerance = 0;
        this._iZone = 0;
    }

    public double getIZone() {
        return _iZone;
    }

    public double getP() {
        return this._kP;
    }
    
    public void setP(double kP) {
        this._kP = kP;
    }

    public double getI() {
        return this._kI;
    }

    public void setI(double kI) {
        this._kI = kI;
    }

    public double getD() {
        return this._kD;
    }

    public void setD(double kD) {
        this._kD = kD;
    }

    public double getF() {
        return this._kF;
    }

    public double getTolerance() {
        return this._tolerance;
    }

    public PIDController createPIDController() {
        return new PIDController(_kP, _kI, _kD);
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("PIDController");
        builder.addDoubleProperty("p", this::getP, this::setP);
        builder.addDoubleProperty("i", this::getI, this::setI);
        builder.addDoubleProperty("d", this::getD, this::setD);
    }
}