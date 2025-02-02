package frc.robot.subsystems.collar;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotContainer.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class CollarSubsystem extends SubsystemBase {
    
    // constructor
    public CollarSubsystem() {
        
    }

    // sets collar motor speed (forward if positive, backward if negative)
    public void runCollarMotor(double speed) {

    }

    // sets intake motor speed to zero and stops motor
    public void collarMotorOff() {

    }

    public Command getCommand(RobotState targetRobotState) {
        return new WaitCommand(0);
    }
}
