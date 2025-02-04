package frc.robot.subsystems.collar;

import frc.robot.RobotContainer.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class CollarSubsystem {

    public Command getCommand(RobotState targetRobotState) {
        return new WaitCommand(0);
    }

}
