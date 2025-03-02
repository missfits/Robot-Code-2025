package frc.robot.subsystems.lifter;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotState;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.function.Supplier;


public class LifterCommandFactory {
    private ElevatorSubsystem m_elevator;
    private ArmSubsystem m_arm;

    // constructor
    public LifterCommandFactory(ElevatorSubsystem elevator, ArmSubsystem arm) {
        m_elevator = elevator;
        m_arm = arm;
    }

    public Command getCommand(RobotState targetRobotState) {
        return new WaitCommand(0);
    }

    public Command moveToCommand(RobotState targetRobotState) {
        if (targetRobotState == RobotState.L4_CORAL) {
            return new ParallelCommandGroup(
                new SequentialCommandGroup( // arm movement 
                    new WaitCommand(3).until(m_elevator.okToMoveArmBackTrigger()), // wait until elevator is sufficiently up
                    m_arm.moveToCommand(targetRobotState.getArmPos())),
                m_elevator.moveToCommand(targetRobotState.getElevatorPos()))
                .until(m_arm.isAtGoal().and(m_elevator.isAtGoal()));
        } else {
            return new ParallelCommandGroup(
                m_arm.moveToCommand(targetRobotState.getArmPos()),
                new SequentialCommandGroup( // elevator movement
                    new WaitCommand(3).until(m_arm.okToMoveElevatorDownTrigger()), // wait until arm is not over the ramp
                    m_elevator.moveToCommand(targetRobotState.getElevatorPos())))
                .until(m_arm.isAtGoal().and(m_elevator.isAtGoal()));
        }
    }
    public Command moveToCommand(Supplier<RobotState> targetRobotStateSupplier) {
        return new SequentialCommandGroup(
            m_arm.moveToCommand(ArmConstants.INITIAL_POSITION),
            m_elevator.moveToCommand(() -> targetRobotStateSupplier.get().getElevatorPos()),
            m_arm.moveToCommand(() -> targetRobotStateSupplier.get().getArmPos())
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
