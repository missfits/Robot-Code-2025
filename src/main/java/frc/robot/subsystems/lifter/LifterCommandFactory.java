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
        if (targetRobotState == RobotState.L4_CORAL) {
            return new ParallelCommandGroup(
                new SequentialCommandGroup( // arm movement 
                    new WaitCommand(3).until(m_elevator.okToMoveArmBackTrigger()), // wait until elevator is sufficiently up
                    m_arm.moveToCommand(targetRobotState.getArmPos())),
                m_elevator.moveToCommand(targetRobotState.getElevatorPos()));
        } else {
            return new ParallelCommandGroup(
                m_arm.moveToCommand(targetRobotState.getArmPos()),
                new SequentialCommandGroup( // elevator movement
                    new WaitCommand(3).until(m_arm.okToMoveElevatorDownTrigger()), // wait until arm is not over the ramp
                    m_elevator.moveToCommand(targetRobotState.getElevatorPos())));
        }
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
