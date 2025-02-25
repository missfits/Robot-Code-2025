package frc.robot.subsystems.collar;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

import frc.robot.Constants.CollarConstants;
import frc.robot.LifterState;

public class CollarSubsystem extends SubsystemBase {
    private final CollarIOHardware m_IO = new CollarIOHardware();
    
    // constructor
    public CollarSubsystem() {
        m_IO.resetPosition();
    }

    public Command getCommand(LifterState targetLifterState) {
        return new WaitCommand(0);
    }

    public Command runCollarOff() {
        return new RunCommand(
            () -> m_IO.setVoltage(0),
            this
        );
    }

    public Command runCollar() {
        return new RunCommand(
            () -> m_IO.setVoltage(CollarConstants.OUTTAKE_MOTOR_SPEED),
            this
        );
    }

    public Command runCollarBackward() {
        return new RunCommand(
            () -> m_IO.setVoltage(-CollarConstants.OUTTAKE_MOTOR_SPEED),
            this
        );
    }
}
