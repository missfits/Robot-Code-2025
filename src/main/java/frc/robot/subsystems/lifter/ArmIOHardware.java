package frc.robot.subsystems.lifter;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.LifterConstants;

import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

public class ArmIOHardware implements ArmIO {
    private final TalonFX m_armMotor = new TalonFX(LifterConstants.ARM_MOTOR_ID);

    // constructor
    public ArmIOHardware() {

    }

    public double getPositionMeters() {
        double angle = m_armMotor.getPosition().getValue().in(Revolutions);
        return angle*LifterConstants.METERS_PER_ROTATION;
    }
}
