package frc.robot.subsystems.collar;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CollarConstants;

public class CollarIOHardware {
    private final TalonFX m_collarMotor = new TalonFX(CollarConstants.COLLAR_MOTOR_ID);

    // constructor
    public CollarIOHardware() {}

    // getters
    public double getPosition() {
        double angle = m_collarMotor.getPosition().getValue().in(Revolutions);
        return angle*CollarConstants.METERS_PER_ROTATION;
    }

    public double getVelocity() {
        double speed = m_collarMotor.getVelocity().getValue().in(RevolutionsPerSecond);
        return speed*CollarConstants.METERS_PER_ROTATION;
    }

    // setters
    public void motorOff() {
        m_collarMotor.setVoltage(0);
        m_collarMotor.stopMotor();
    }

    public void setPosition(double value) {
        m_collarMotor.setPosition(value);
    }

    public void resetPosition() {
        setPosition(0);
    }

    public void setVoltage(double value) {
        m_collarMotor.setControl(new VoltageOut(value));
    }
    
    public void setVoltage(PositionVoltage request) {
        m_collarMotor.setControl(request);
    }
}
