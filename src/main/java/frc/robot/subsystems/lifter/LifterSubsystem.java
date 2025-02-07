package frc.robot.subsystems.lifter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.RobotContainer.RobotState;


public class LifterSubsystem extends SubsystemBase {
    private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
    private final ArmSubsystem m_arm = new ArmSubsystem();

    // constructor
    public LifterSubsystem() {}

    public Command getCommand(RobotState targetRobotState) {
        return new WaitCommand(0);
    }

    public Command moveToCommand(RobotState targetRobotState) {
        return new ParallelCommandGroup(
            m_elevator.moveToCommand(targetRobotState.getElevatorPos()),
            m_arm.moveToCommand(targetRobotState.getArmPos())
        );
    }

}
