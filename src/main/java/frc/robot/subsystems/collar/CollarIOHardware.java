package frc.robot.subsystems.collar;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import frc.robot.Constants.CollarConstants;

public class CollarIOHardware {
    private final TalonFX m_collarMotor = new TalonFX(CollarConstants.COLLAR_MOTOR_ID);
    private final StatusSignal<Angle> m_positionSignal = m_collarMotor.getPosition();
    private final StatusSignal<AngularVelocity> m_velocitySignal = m_collarMotor.getVelocity();

    // constructor
    public CollarIOHardware() {
        var talonFXConfigurator = m_collarMotor.getConfigurator();
        var limitConfigs = new CurrentLimitsConfigs();

        limitConfigs.StatorCurrentLimit = CollarConstants.MOTOR_STATOR_LIMIT;
        limitConfigs.StatorCurrentLimitEnable = true;

        talonFXConfigurator.apply(limitConfigs);
    }

    // getters
    public double getPosition() {
        return m_positionSignal.refresh().getValue().in(Revolutions)*CollarConstants.METERS_PER_ROTATION;
    }

    public double getVelocity() {
        return m_velocitySignal.refresh().getValue().in(RevolutionsPerSecond)*CollarConstants.METERS_PER_ROTATION;
    }

    // setters
    public void motorOff() {
        m_collarMotor.stopMotor();
    }

    public void setPosition(double value) {
        m_collarMotor.setPosition(value);
    }

    public void resetPosition() {
        setPosition(0);
    }

    public void setVoltage(double value) {
        SmartDashboard.putNumber("collar/voltage", value);
        m_collarMotor.setControl(new VoltageOut(value));
    }
    
    public void requestClosedLoopPosition(double value) {
        m_collarMotor.setControl(new PositionVoltage(value));
    }
}
