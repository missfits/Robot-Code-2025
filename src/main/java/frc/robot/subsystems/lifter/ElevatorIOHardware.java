package frc.robot.subsystems.lifter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.*;

import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOHardware {
    private final TalonFX m_elevatorMotor = new TalonFX(ElevatorConstants.ELEVATOR_MOTOR_ID);
    
    // constructor
    public ElevatorIOHardware() {}

    // ----- MOTOR METHODS -----
    // getters
    public double getPosition() {
        double angle = m_elevatorMotor.getPosition().getValue().in(Revolutions);
        return angle*ElevatorConstants.METERS_PER_ROTATION;
    }

    public double getVelocity() {
        double speed = m_elevatorMotor.getVelocity().getValue().in(RevolutionsPerSecond);
        return speed*ElevatorConstants.METERS_PER_ROTATION;
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
    
    public void setVoltage(PositionVoltage request) {
        m_elevatorMotor.setControl(request);
    }
}
