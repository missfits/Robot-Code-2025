package frc.robot.subsystems.collar;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.CollarConstants;


public class CollarCommandFactory {
    private CollarSubsystem m_collar;
    private RampSensorSubsystem m_rampSensors;

    // constructor
    public CollarCommandFactory(CollarSubsystem collar, RampSensorSubsystem rampSensors) {
        m_collar = collar;
        m_rampSensors = rampSensors;
    }
    
    public Command intakeCoralSequence() {
        return Commands.sequence(
            m_collar.runCollar(CollarConstants.INTAKE_MOTOR_SPEED).until(m_rampSensors.coralSeenAfterRamp()),
            m_collar.runCollar(CollarConstants.INTAKE_SECONDARY_MOTOR_SPEED).until(m_rampSensors.coralSeenAfterRamp().negate()),
            m_collar.runCollar(CollarConstants.INTAKE_SECONDARY_BACK_MOTOR_SPEED).withTimeout(0.3),
            m_collar.runCollarOffInstant());
    }

    public Command runCollarOut() {
        return m_collar.runCollar(CollarConstants.OUTTAKE_MOTOR_SPEED);
    }

    public Command runCollarInSecondary() {
        return m_collar.runCollar(CollarConstants.INTAKE_SECONDARY_MOTOR_SPEED);
    }

    public Command runCollarIn() {
        return m_collar.runCollar(CollarConstants.INTAKE_MOTOR_SPEED);
    }

    public Command runCollarBackwards() {
        return m_collar.runCollar(CollarConstants.BACKWARDS_MOTOR_SPEED);
    }
}
