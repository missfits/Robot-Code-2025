package frc.robot.subsystems.lifter;

import frc.robot.RobotContainer.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;


public class ElevatorAndArmSubsystem extends SubsystemBase {
    
    public Command getCommand(RobotState targetRobotState) {
        return new WaitCommand(0);
    }

}
