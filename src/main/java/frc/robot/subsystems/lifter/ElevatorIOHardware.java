package frc.robot.subsystems.lifter;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOHardware {
    private final TalonFX m_elevatorMotor = new TalonFX(ElevatorConstants.ELEVATOR_MOTOR_ID);
    private final StatusSignal<Angle> m_positionSignal = m_elevatorMotor.getPosition();
    private final StatusSignal<AngularVelocity> m_velocitySignal = m_elevatorMotor.getVelocity();

    // constructor
    public ElevatorIOHardware() {
        var talonFXConfigurator = m_elevatorMotor.getConfigurator();
        var limitConfigs = new CurrentLimitsConfigs();

        limitConfigs.StatorCurrentLimit = ElevatorConstants.MOTOR_STATOR_LIMIT;
        limitConfigs.StatorCurrentLimitEnable = true;

        talonFXConfigurator.apply(limitConfigs);
    }

    // ----- MOTOR METHODS -----
    // getters
    public double getPosition() {
        return m_positionSignal.refresh().getValue().in(Revolutions)*ElevatorConstants.METERS_PER_ROTATION;
    }

    public double getVelocity() {
        return m_velocitySignal.refresh().getValue().in(RevolutionsPerSecond)*ElevatorConstants.METERS_PER_ROTATION;
    }

    // setters
    public void motorOff() {
        m_elevatorMotor.setVoltage(0);
        m_elevatorMotor.stopMotor();
    }

    public void setPosition(double value) {
        m_elevatorMotor.setPosition(value);
    }

    public void resetPosition() {
        setPosition(0);
    }

    public void setVoltage(double value) {
        m_elevatorMotor.setControl(new VoltageOut(value));
    }
    
    public void requestClosedLoopPosition(double value) {
        m_elevatorMotor.setControl(new PositionVoltage(value));
    }
}
