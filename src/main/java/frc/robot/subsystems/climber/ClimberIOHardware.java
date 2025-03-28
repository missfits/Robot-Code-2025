package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClimberConstants;


public class ClimberIOHardware {
    private final TalonFX m_climberMotor = new TalonFX(ClimberConstants.CLIMBER_MOTOR_ID);
    private final StatusSignal<Angle> m_positionSignal = m_climberMotor.getPosition();
    private final StatusSignal<AngularVelocity> m_velocitySignal = m_climberMotor.getVelocity();

    // constructor
    public ClimberIOHardware() {
        var talonFXConfigurator = m_climberMotor.getConfigurator();
        var limitConfigs = new CurrentLimitsConfigs();

        limitConfigs.StatorCurrentLimit = ClimberConstants.MOTOR_STATOR_LIMIT;
        limitConfigs.StatorCurrentLimitEnable = true;

        talonFXConfigurator.apply(limitConfigs);
    }

    // getters
    public double getPosition() {
        return Math.toRadians(m_positionSignal.refresh().getValue().in(Revolutions)*ClimberConstants.DEGREES_PER_ROTATION);
    }

    public double getPositionDegrees() {
        return m_positionSignal.refresh().getValue().in(Revolutions)*ClimberConstants.DEGREES_PER_ROTATION;
    }

    public double getVelocity() {
        return Math.toRadians(m_velocitySignal.refresh().getValue().in(RevolutionsPerSecond)*ClimberConstants.DEGREES_PER_ROTATION);
    }

    public double getVelocityDegrees() {
        return m_velocitySignal.refresh().getValue().in(RevolutionsPerSecond)*ClimberConstants.DEGREES_PER_ROTATION;
    }

    // setters
    public void motorOff() {
        m_climberMotor.stopMotor();
    }

    public void setPosition(double value) {
        m_climberMotor.setPosition(value);
    }

    public void setVoltage(double value) {
        m_climberMotor.setControl(new VoltageOut(value));
    }
    
    public void requestClosedLoopPosition(double value) {
        m_climberMotor.setControl(new PositionVoltage(value));
    }

    public void setBrake(boolean brake) {
        m_climberMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    public void setCoast() {
        m_climberMotor.setControl(new CoastOut());
    }
}
