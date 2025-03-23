package frc.robot.subsystems.lifter;

import edu.wpi.first.math.MathUtil;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import frc.robot.Constants.ArmConstants;

public class ArmIOHardware {
    private final TalonFX m_armMotor = new TalonFX(ArmConstants.ARM_MOTOR_ID);
    private final StatusSignal<Angle> m_positionSignal = m_armMotor.getPosition();
    private final StatusSignal<AngularVelocity> m_velocitySignal = m_armMotor.getVelocity();
    private final StatusSignal<Voltage> m_voltageSignal = m_armMotor.getMotorVoltage();
    private final StatusSignal<Double> m_targetPositionSignal = m_armMotor.getClosedLoopReference();
    private final StatusSignal<Double> m_targetVelocitySignal = m_armMotor.getClosedLoopReferenceSlope();


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

    public double getTargetPosition() {
        return Math.toRadians(m_targetPositionSignal.refresh().getValue()*ArmConstants.DEGREES_PER_ROTATION);
    }

    public double getTargetPositionDegrees() {
        return m_targetPositionSignal.refresh().getValue()*ArmConstants.DEGREES_PER_ROTATION;
    }

    public double getVelocity() {
        return Math.toRadians(m_velocitySignal.refresh().getValue().in(RevolutionsPerSecond)*ArmConstants.DEGREES_PER_ROTATION);
    }

    public double getVelocityDegrees() {
        return m_velocitySignal.refresh().getValue().in(RevolutionsPerSecond)*ArmConstants.DEGREES_PER_ROTATION;
    }

    public double getTargetVelocity() {
        return Math.toRadians(m_targetVelocitySignal.refresh().getValue()*ArmConstants.DEGREES_PER_ROTATION);
    }

    public double getTargetVelocityDegrees() {
        return m_targetVelocitySignal.refresh().getValue()*ArmConstants.DEGREES_PER_ROTATION;
    }

    public double getVoltage() {
        return m_voltageSignal.refresh().getValue().in(Volts);
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
        // value = MathUtil.clamp(value, -, 8);
        m_armMotor.setControl(new VoltageOut(value));
    }
    
    public void requestClosedLoopPosition(double value) {
        m_armMotor.setControl(new PositionVoltage(value));
    }

    public void setBrake(boolean brake) {
        m_armMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    public void config(TalonFXConfiguration configs) {
        m_armMotor.getConfigurator().apply(configs);
    }

    public void setControl(ControlRequest request) {
        m_armMotor.setControl(request);
    }
}

