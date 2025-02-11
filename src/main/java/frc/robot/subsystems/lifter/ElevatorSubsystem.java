package frc.robot.subsystems.lifter;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase{
    private final ElevatorIOHardware m_IO = new ElevatorIOHardware();
    private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(
        ElevatorConstants.kS,
        ElevatorConstants.kG,
        ElevatorConstants.kV,
        ElevatorConstants.kA
    );
    private final PIDController m_controller = new PIDController(
        ElevatorConstants.kP,
        ElevatorConstants.kI,
        ElevatorConstants.kD
    );
    private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(
        ElevatorConstants.kMaxV, ElevatorConstants.kMaxA
    );
    private TrapezoidProfile.State m_goal;
    private TrapezoidProfile.State m_profiledReference;
    private TrapezoidProfile m_profile;

    // constructor
    public ElevatorSubsystem() {}

    // commands
    public Command manualMoveCommand() {
        return new StartEndCommand(
            () -> m_IO.setVoltage(ElevatorConstants.MANUAL_MOVE_MOTOR_SPEED),
            () -> m_IO.motorOff(),
            this
        );
    }

    public Command moveToCommand(double targetPosition) {
        return moveToCommand(new TrapezoidProfile.State(targetPosition, 0));
    }

    public Command moveToCommand(TrapezoidProfile.State goal) {
        return new FunctionalCommand(
            () -> initalizeMoveTo(goal),
            () -> executeMoveTo(),
            interrupted -> m_IO.motorOff(),
            () -> m_profile.isFinished(0.02),
            this
        );
    }

    // helper commands
    private void initalizeMoveTo(TrapezoidProfile.State goal) {
        m_goal = goal;
        m_profiledReference = new TrapezoidProfile.State(m_IO.getPosition(), m_IO.getVelocity());
        m_profile = new TrapezoidProfile(m_constraints);
    }

    private void executeMoveTo() {
        // recalculate the profiled reference point (the vel + pos that we want)
        m_profiledReference = m_profile.calculate(0.02, m_profiledReference, m_goal);
        
        // calculate part of the power based on target velocity 
        double feedForwardPower = m_feedforward.calculate(m_profiledReference.velocity);

        // calculate part of the power based on target position + current position
        double PIDPower = m_controller.calculate(m_IO.getPosition(), m_profiledReference.position);

        m_IO.setVoltage(feedForwardPower + PIDPower);
    }
}
