package frc.robot.subsystems.lifter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotContainer.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class LifterSubsystem extends SubsystemBase {
    
    // constructor
    public LifterSubsystem() {

    }

    public Command getCommand(RobotState targetRobotState) {
        return new WaitCommand(0);
    }
}
