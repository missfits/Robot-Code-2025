package frc.robot.subsystems.lifter;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.RobotState;


public class LifterCommandFactory {
    private ElevatorSubsystem m_elevator;
    private ArmSubsystem m_arm;

    private enum SelectorCases {
        TO_L4,
        TO_INTAKE_FROM_BAD_POS,
        TO_L1_FROM_BAD_POS,
        ALL_OK
    }

    // constructor
    public LifterCommandFactory(ElevatorSubsystem elevator, ArmSubsystem arm) {
        m_elevator = elevator;
        m_arm = arm;
    }

    public Command getCommand(RobotState targetRobotState) {
        return new WaitCommand(0);
    }

    private SelectorCases selectMoveToCommand(RobotState targetRobotState) {
        if (targetRobotState == RobotState.L4_CORAL) { 
            return SelectorCases.TO_L4;
        } if ((targetRobotState == RobotState.INTAKE) && !m_elevator.okToMoveArmBack()) {
            return SelectorCases.TO_INTAKE_FROM_BAD_POS;
        } if ((targetRobotState == RobotState.L1_CORAL) && !m_elevator.okToMoveArmBack()) {
            return SelectorCases.TO_L1_FROM_BAD_POS;
        } else {
            return SelectorCases.ALL_OK;
        }
    }

    private Command toL4Command(RobotState targetRobotState) {
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

    private Command toIntakeFromBadPosCommand(RobotState targetRobotState) {
        return new SequentialCommandGroup(
            // run arm to ok position (a little past INTAKE)
            new ParallelDeadlineGroup(
                m_arm.moveToCommand(ArmConstants.INTERMEDIATE_POS_ELEVATOR_CLEAR).until(m_arm.okToMoveElevatorDownTrigger()),
                m_elevator.keepInPlaceCommand()),
            // move elevator down
            new ParallelDeadlineGroup(
                m_elevator.moveToCommand(targetRobotState.getElevatorPos()).until(m_elevator.isAtGoal()),
                m_arm.keepInPlaceCommand()),

            // move arm back to INTAKE
            new ParallelDeadlineGroup(
                m_arm.moveToCommand(targetRobotState.getArmPos()).until(m_arm.isAtGoal()), 
                m_elevator.keepInPlaceCommand()));
        // deadline groups so keep in place command will run during the sequence 
    }

    private Command toL1FromBadPosCommand(RobotState targetRobotState) {   
        return new SequentialCommandGroup(
            // move arm to an ok position
            new ParallelDeadlineGroup(
                m_arm.moveToCommand(ArmConstants.INTERMEDIATE_POS_ELEVATOR_CLEAR).until(m_arm.okToMoveElevatorDownTrigger()),
                m_elevator.keepInPlaceCommand()),
            // then move arm and elevator at same time to L1
            new ParallelCommandGroup(
                m_elevator.moveToCommand(targetRobotState.getElevatorPos()).until(m_elevator.isAtGoal()).asProxy(),
                m_arm.moveToCommand(targetRobotState.getArmPos()).until(m_arm.isAtGoal()).asProxy())); 
        // deadline group so keep in place command will run during the sequence 
    }

    private Command allOkMoveToCommand(RobotState targetRobotState) {
        return new ParallelCommandGroup(
            m_arm.moveToCommand(targetRobotState.getArmPos()), 
            m_elevator.moveToCommand(targetRobotState.getElevatorPos()))
            .until(m_arm.isAtGoal().and(m_elevator.isAtGoal()));
    }

    public Command moveToCommand(RobotState targetRobotState) {
        return new SelectCommand<SelectorCases>(
            Map.ofEntries(
              Map.entry(SelectorCases.TO_L4, toL4Command(targetRobotState)),
              Map.entry(SelectorCases.TO_INTAKE_FROM_BAD_POS, toIntakeFromBadPosCommand(targetRobotState)),
              Map.entry(SelectorCases.TO_L1_FROM_BAD_POS, toL1FromBadPosCommand(targetRobotState)),
              Map.entry(SelectorCases.ALL_OK, allOkMoveToCommand(targetRobotState))),
            () -> selectMoveToCommand(targetRobotState));
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
