package frc.robot.subsystems.lifter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOHardware {
    private final TalonFX m_elevatorMotor = new TalonFX(ElevatorConstants.ELEVATOR_MOTOR_ID);
    private final StatusSignal<Angle> m_positionSignal = m_elevatorMotor.getPosition();
    private final StatusSignal<AngularVelocity> m_velocitySignal = m_elevatorMotor.getVelocity();
    private final StatusSignal<Current> m_currentSignal = m_elevatorMotor.getStatorCurrent();
    private final StatusSignal<Voltage> m_voltageSignal = m_elevatorMotor.getMotorVoltage();
    private final StatusSignal<Double> m_targetPositionSignal = m_elevatorMotor.getClosedLoopReference();
    private final StatusSignal<Double> m_targetVelocitySignal = m_elevatorMotor.getClosedLoopReferenceSlope();


    // constructor
    public ElevatorIOHardware() {
        StatusSignal.setUpdateFrequencyForAll(100, m_positionSignal, m_velocitySignal, m_voltageSignal, m_targetPositionSignal, m_targetVelocitySignal);


        var talonFXConfigurator = m_elevatorMotor.getConfigurator();
        var limitConfigs = new CurrentLimitsConfigs();

        limitConfigs.StatorCurrentLimit = ElevatorConstants.MOTOR_STATOR_LIMIT;
        limitConfigs.StatorCurrentLimitEnable = true;

        talonFXConfigurator.apply(limitConfigs);

        resetSlot0Gains();
    }

    // ----- MOTOR METHODS -----
    // getters
    public double getPosition() {
        return m_positionSignal.refresh().getValue().in(Revolutions)*ElevatorConstants.METERS_PER_ROTATION;
    }

    public double getTargetPosition() {
        return m_targetPositionSignal.refresh().getValue()*ElevatorConstants.METERS_PER_ROTATION;
    }

    public double getVelocity() {
        return m_velocitySignal.refresh().getValue().in(RevolutionsPerSecond)*ElevatorConstants.METERS_PER_ROTATION;
    }

    public double getCurrent() {
        return m_currentSignal.refresh().getValue().in(Amp);
    }
    
    public double getTargetVelocity() {
        return m_targetVelocitySignal.refresh().getValue()*ElevatorConstants.METERS_PER_ROTATION;
    }

    public double getVoltage() {
        return m_voltageSignal.refresh().getValue().in(Volts);
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
            value = MathUtil.clamp(value, 0, 1000);
        }
        // if position is too high, only run the elevator down (or in placd)
        if (this.getPosition() > ElevatorConstants.POSITION_UPPER_LIMIT) {
            value = MathUtil.clamp(value, -1000, ElevatorConstants.kG);

        }

        m_elevatorMotor.setControl(new VoltageOut(value));
    }

    public void setVoltageNoCheck(double value) {
        m_elevatorMotor.setControl(new VoltageOut(value));
        SmartDashboard.putNumber("elevator/voltage", value);
    } 
    
    public void requestClosedLoopPosition(double value) {
        m_elevatorMotor.setControl(new PositionVoltage(value));
    }

    public void setBrake(boolean brake) {
        m_elevatorMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    public void setClosedLoopPositionVoltage(double position, double feedfowardVoltage) {
        m_elevatorMotor.setControl(new PositionVoltage(position/ElevatorConstants.METERS_PER_ROTATION).withFeedForward(feedfowardVoltage).withSlot(0));
    }

    public void resetSlot0Gains() {
        var talonFXConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = ElevatorConstants.kS*ElevatorConstants.METERS_PER_ROTATION; 
        slot0Configs.kV = ElevatorConstants.kV*ElevatorConstants.METERS_PER_ROTATION; 
        slot0Configs.kA = ElevatorConstants.kA*ElevatorConstants.METERS_PER_ROTATION; 
        slot0Configs.kP = ElevatorConstants.kP*ElevatorConstants.METERS_PER_ROTATION;
        slot0Configs.kI = ElevatorConstants.kI*ElevatorConstants.METERS_PER_ROTATION;
        slot0Configs.kD = ElevatorConstants.kD*ElevatorConstants.METERS_PER_ROTATION;

        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ElevatorConstants.kMaxV*ElevatorConstants.METERS_PER_ROTATION; 
        motionMagicConfigs.MotionMagicAcceleration = ElevatorConstants.kMaxA*ElevatorConstants.METERS_PER_ROTATION; 
        motionMagicConfigs.MotionMagicJerk = 0; // no jerk limit

        m_elevatorMotor.getConfigurator().apply(talonFXConfigs);
    }

    public void setCoast() {
        m_elevatorMotor.setControl(new CoastOut());
    }
}
