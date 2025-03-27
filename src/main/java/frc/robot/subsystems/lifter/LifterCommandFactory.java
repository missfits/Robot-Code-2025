package frc.robot.subsystems.lifter;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotState;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.RobotStateConstants;
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
            return Commands.parallel(
                Commands.sequence(
                    m_arm.moveToCommand(ArmConstants.MIN_POS_ELEVATOR_CLEAR)
                        .until(m_elevator.okToMoveArmBackTrigger()),
                    m_arm.moveToCommand(ArmConstants.UPPER_INSIDE_ROBOT_BOUND)
                        .until(m_elevator.isAtGoal(targetRobotState.getElevatorPos())),
                    m_arm.moveToCommand(targetRobotState.getArmPos())
                ),
                Commands.sequence(
                    Commands.waitSeconds(3).until(m_arm.isArmInsideRobotTrigger()),
                    m_elevator.moveToCommand(targetRobotState.getElevatorPos())
                )
            ).until(isLifterAtGoal(targetRobotState.getArmPos(), targetRobotState.getElevatorPos()));
        } else {
            // move arm to intermediate pos inside the robot before moving elevator
            double armIntermediatePosition = MathUtil.clamp(targetRobotState.getArmPos(), 
                ArmConstants.MIN_POS_ELEVATOR_CLEAR, ArmConstants.LOWER_INSIDE_ROBOT_BOUND);
            // keep elevator at intermediate pos to let arm move past ramp
            double elevatorIntermediatePosition = Math.max(targetRobotState.getElevatorPos(), 
                ElevatorConstants.MIN_POS_ARM_CLEAR);
            return Commands.parallel(
                Commands.sequence(
                    m_arm.moveToCommand(armIntermediatePosition)
                    .until(m_arm.isArmInsideRobotTrigger()
                        .and(m_elevator.isAtGoal(targetRobotState.getElevatorPos()))),
                    m_arm.moveToCommand(targetRobotState.getArmPos())
                ), 
                Commands.sequence(
                    Commands.waitSeconds(3).until(m_arm.isArmInsideRobotTrigger()),
                    m_elevator.moveToCommand(elevatorIntermediatePosition)
                        .until(m_arm.okToMoveElevatorDownTrigger()),
                    m_elevator.moveToCommand(targetRobotState.getElevatorPos())
                )
            ).until(isLifterAtGoal(targetRobotState.getArmPos(), targetRobotState.getElevatorPos()));
        }
    }

    public Command moveToCommand(Supplier<RobotState> targetRobotStateSupplier) {
        return new SequentialCommandGroup(
            m_arm.moveToCommand(ArmConstants.INITIAL_POSITION),
            m_elevator.moveToCommand(() -> targetRobotStateSupplier.get().getElevatorPos()),
            m_arm.moveToCommand(() -> targetRobotStateSupplier.get().getArmPos())
        );
    }

    private Trigger isLifterAtGoal(double armGoal, double elevatorGoal) {
        return m_arm.isAtGoal(armGoal).and(m_elevator.isAtGoal(elevatorGoal));
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
