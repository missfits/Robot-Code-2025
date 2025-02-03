package frc.robot.subsystems.ramp;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer.RobotState;

public class RampSubsystem {

    // constructor
    public RampSubsystem() {
        
    }

    public Command getCommand(RobotState targetRobotState) {
        return new WaitCommand(0);
    }
}
