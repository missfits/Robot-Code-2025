package frc.robot.subsystems.lifter;

import edu.wpi.first.math.MathUtil;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import frc.robot.Constants.ArmConstants;

public class ArmIOHardware {
    private final TalonFX m_armMotor = new TalonFX(ArmConstants.ARM_MOTOR_ID);
    private final StatusSignal<Angle> m_positionSignal = m_armMotor.getPosition();
    private final StatusSignal<AngularVelocity> m_velocitySignal = m_armMotor.getVelocity();

    // constructor
    public ArmIOHardware() {
        var talonFXConfigurator = m_armMotor.getConfigurator();
        var limitConfigs = new CurrentLimitsConfigs();

        limitConfigs.StatorCurrentLimit = ArmConstants.MOTOR_STATOR_LIMIT;
        limitConfigs.StatorCurrentLimitEnable = true;

        talonFXConfigurator.apply(limitConfigs);
    }

    // getters
    public double getPosition() {
        return Math.toRadians(m_positionSignal.refresh().getValue().in(Revolutions)*ArmConstants.DEGREES_PER_ROTATION);
    }

    public double getPositionDegrees() {
        return m_positionSignal.refresh().getValue().in(Revolutions)*ArmConstants.DEGREES_PER_ROTATION;
    }

    public double getVelocity() {
        return m_velocitySignal.refresh().getValue().in(RevolutionsPerSecond)*ArmConstants.DEGREES_PER_ROTATION;
    }

    // setters
    public void motorOff() {
        m_armMotor.stopMotor();
    }

    public void setPosition(double value) {
        m_armMotor.setPosition(value);
    }

    public void resetPosition() {
        setPosition(ArmConstants.INITIAL_POSITION);
    }

    public void setVoltage(double value) {
        value = MathUtil.clamp(value, -3, 3);
        m_armMotor.setControl(new VoltageOut(value));
        SmartDashboard.putNumber("arm/voltage", value);
    }
    
    public void requestClosedLoopPosition(double value) {
        m_armMotor.setControl(new PositionVoltage(value));
    }

    public void setBrake(boolean brake) {
        m_armMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }
}
