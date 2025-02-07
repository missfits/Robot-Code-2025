package frc.robot.subsystems.collar;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotContainer.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class CollarSubsystem extends SubsystemBase {
    private final CollarIOHardware m_IO = new CollarIOHardware();
    
    // constructor
    public CollarSubsystem() {
        
    }

    public Command getCommand(RobotState targetRobotState) {
        return new WaitCommand(0);
    }
}
