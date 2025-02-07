package frc.robot.subsystems.lifter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import frc.robot.Constants.ArmConstants;

public class ArmIOHardware {
    private final TalonFX m_armMotor = new TalonFX(ArmConstants.ARM_MOTOR_ID);

    // constructor
    public ArmIOHardware() {}

    // getters
    public double getPosition() {
        double angle = m_armMotor.getPosition().getValue().in(Revolutions);
        return angle*ArmConstants.DEGREES_PER_ROTATION;
    }

    public double getVelocity() {
        double speed = m_armMotor.getVelocity().getValue().in(RevolutionsPerSecond);
        return speed*ArmConstants.DEGREES_PER_ROTATION;
    }

    // setters
    public void motorOff() {
        m_armMotor.setVoltage(0);
        m_armMotor.stopMotor();
    }

    public void setPosition(double value) {
        m_armMotor.setPosition(value);
    }

    public void resetPosition() {
        setPosition(0);
    }

    public void setVoltage(double value) {
        m_armMotor.setControl(new VoltageOut(value));
    }
    
    public void setVoltage(PositionVoltage request) {
        m_armMotor.setControl(request);
    }
}
