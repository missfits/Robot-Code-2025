package frc.robot.subsystems.lifter;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.LifterConstants;

import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

public class ElevatorIOHardware implements ElevatorIO {
    private final TalonFX m_elevatorMotor = new TalonFX(LifterConstants.ELEVATOR_MOTOR_ID);
    
    // constructor
    public ElevatorIOHardware() {

    }

    public double getPositionMeters() {
        double angle = m_elevatorMotor.getPosition().getValue().in(Revolutions);
        return angle*LifterConstants.METERS_PER_ROTATION;
    }
}
