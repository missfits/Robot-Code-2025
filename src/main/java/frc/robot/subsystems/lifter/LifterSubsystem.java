package frc.robot.subsystems.lifter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.RobotContainer.RobotState;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ArmConstants;


public class LifterSubsystem extends SubsystemBase {
    private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
    private final ArmSubsystem m_arm = new ArmSubsystem();

    // constructor
    public LifterSubsystem() {}

    public Command getCommand(RobotState targetRobotState) {
        return new WaitCommand(0);
    }

    public Command moveToCommand(RobotState targetRobotState) {
        return new WaitCommand(0);
    }

}
