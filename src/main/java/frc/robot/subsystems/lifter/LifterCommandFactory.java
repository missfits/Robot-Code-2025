package frc.robot.subsystems.lifter;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.RobotState;


public class LifterCommandFactory {
    private ElevatorSubsystem m_elevator;
    private ArmSubsystem m_arm;

    private BooleanSupplier armInBadPositionSupplier = () -> !m_elevator.okToMoveArmBack();

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
                new SequentialCommandGroup( 
                    // arm movement: while waiting until elevator is up, move arm to a clear position 
                    m_arm.moveToCommand(ArmConstants.INTERMEDIATE_POS_ELEVATOR_CLEAR).until(m_elevator.okToMoveArmBackTrigger()),
                    m_arm.moveToCommand(targetRobotState.getArmPos())),
                new SequentialCommandGroup(
                    // elevator movement: wait until arm is out of the way
                    new WaitCommand(3).until(m_arm.okToMoveElevatorDownTrigger()),
                    m_elevator.moveToCommand(targetRobotState.getElevatorPos())))
                .until(m_arm.isAtGoal().and(m_elevator.isAtGoal())); 
        } 

        if ((targetRobotState == RobotState.INTAKE) && armInBadPositionSupplier.getAsBoolean()) {
            return new SequentialCommandGroup(
                // run arm to ok position (a little past INTAKE), move elevator down, then move arm back to INTAKE
                m_arm.moveToCommand(ArmConstants.INTERMEDIATE_POS_ELEVATOR_CLEAR).until(m_arm.okToMoveElevatorDownTrigger()).asProxy(),
                m_elevator.moveToCommand(targetRobotState.getElevatorPos()).until(m_elevator.isAtGoal()).asProxy(),
                m_arm.moveToCommand(targetRobotState.getArmPos()).until(m_arm.isAtGoal()).asProxy());
            // proxied everything so keep in place command will run during the sequence
        }
        
        if ((targetRobotState == RobotState.L1_CORAL) && armInBadPositionSupplier.getAsBoolean()) {
            return new SequentialCommandGroup(
                // move arm to an ok position, then move arm and elevator at same time to L1
                m_arm.moveToCommand(ArmConstants.INTERMEDIATE_POS_ELEVATOR_CLEAR).until(m_arm.okToMoveElevatorDownTrigger()).asProxy(),
                new ParallelCommandGroup(
                    m_elevator.moveToCommand(targetRobotState.getElevatorPos()).until(m_elevator.isAtGoal()).asProxy(),
                    m_arm.moveToCommand(targetRobotState.getArmPos()).until(m_arm.isAtGoal()).asProxy())); 
            // proxied everything so keep in place command will run during the sequence
        } 
        
        // otherwise, parallel movement 
        else {
            return new ParallelCommandGroup(
                m_arm.moveToCommand(targetRobotState.getArmPos()), 
                m_elevator.moveToCommand(targetRobotState.getElevatorPos()))
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
