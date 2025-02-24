package frc.robot.subsystems.lifter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotState;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class LifterCommandFactory {
    private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
    private final ArmSubsystem m_arm = new ArmSubsystem();

    // constructor
    public LifterCommandFactory() {}

    public Command getCommand(RobotState targetRobotState) {
        return new WaitCommand(0);
    }

    public Command moveToCommand(RobotState targetRobotState) {
        return new SequentialCommandGroup(
            m_arm.moveToCommand(ArmConstants.INITIAL_POSITION),
            m_elevator.moveToCommand(targetRobotState.getElevatorPos()),
            m_arm.moveToCommand(targetRobotState.getArmPos())
        );
    }

    public void resetControllers() {
        m_elevator.resetControllers();
        m_arm.resetControllers();

    }

    public void setDisabledNeutralMode() {
        m_arm.setBrake(false);
        m_elevator.setBrake(true);
    }

    public void setEnabledNeutralMode() {
        m_arm.setBrake(true);
        m_elevator.setBrake(true);
        
    }
}
