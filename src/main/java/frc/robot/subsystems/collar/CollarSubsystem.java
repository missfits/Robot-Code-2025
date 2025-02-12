package frc.robot.subsystems.collar;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotState;

public class CollarSubsystem extends SubsystemBase {
    private final CollarIOHardware m_IO = new CollarIOHardware();
    
    // constructor
    public CollarSubsystem() {
        m_IO.resetPosition();
    }

    public Command getCommand(RobotState targetRobotState) {
        return new WaitCommand(0);
    }
}
