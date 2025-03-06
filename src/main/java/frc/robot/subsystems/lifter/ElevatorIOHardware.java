package frc.robot.subsystems.lifter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
        m_elevatorMotor.stopMotor();
    }

    public void setPosition(double value) {
        m_elevatorMotor.setPosition(value);
    }

    public void resetPosition() {
        setPosition(ElevatorConstants.INITIAL_POSITION);
    }

    public void setVoltage(double value) {
        value = MathUtil.clamp(value, -8, 8);

        // if position is too low, only run the elevator up 
        if (this.getPosition() < ElevatorConstants.POSITION_LOWER_LIMIT) {
            value = MathUtil.clamp(value, -0.05, 1000);
        }
        // if position is too high, only run the elevator down (or in placd)
        if (this.getPosition() > ElevatorConstants.POSITION_UPPER_LIMIT) {
            value = MathUtil.clamp(value, -1000, ElevatorConstants.kG);

        }

        m_elevatorMotor.setControl(new VoltageOut(value));
        SmartDashboard.putNumber("elevator/voltage", value);
    }
    
    public void requestClosedLoopPosition(double value) {
        m_elevatorMotor.setControl(new PositionVoltage(value));
    }

    public void setBrake(boolean brake) {
        m_elevatorMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }
}
