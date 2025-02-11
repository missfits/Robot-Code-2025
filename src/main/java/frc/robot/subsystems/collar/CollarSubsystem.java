package frc.robot.subsystems.collar;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

import frc.robot.Constants.CollarConstants;
import frc.robot.RobotContainer.RobotState;

public class CollarSubsystem extends SubsystemBase {
    private final CollarIOHardware m_IO = new CollarIOHardware();
    
    // constructor
    public CollarSubsystem() {
        
    }

    public Command getCommand(RobotState targetRobotState) {
        return new WaitCommand(0);
    }

    public Command runCollar() {
        return new StartEndCommand(
            () -> m_IO.setVoltage(CollarConstants.OUTTAKE_MOTOR_SPEED),
            () -> m_IO.motorOff(),
            this
        );
    }
}
