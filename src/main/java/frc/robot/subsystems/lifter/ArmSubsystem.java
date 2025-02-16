package frc.robot.subsystems.lifter;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    private final ArmIOHardware m_IO = new ArmIOHardware();
    private final ArmFeedforward m_feedforward = new ArmFeedforward(
        ArmConstants.kS,
        ArmConstants.kG,
        ArmConstants.kV,
        ArmConstants.kA
    );
    private final PIDController m_controller = new PIDController(
        ArmConstants.kP,
        ArmConstants.kI,
        ArmConstants.kD
    );
    private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(
        ArmConstants.kMaxV, ArmConstants.kMaxA
    );
    private TrapezoidProfile.State m_goal;
    private TrapezoidProfile.State m_profiledReference;
    private TrapezoidProfile m_profile;

    // constructor
    public ArmSubsystem() {
        m_IO.resetPosition();
        m_controller.enableContinuousInput(0, 360);
    }

    // commands
    public Command keepInPlaceCommand() {
        return new StartEndCommand(
            () -> m_IO.setVoltage(m_feedforward.calculate(m_IO.getPosition() - ArmConstants.POSITION_OFFSET, 0)),
            () -> m_IO.motorOff(),
            this
        );
    }

    public Command manualMoveCommand() {
        return new StartEndCommand(
            () -> m_IO.setVoltage(ArmConstants.MANUAL_MOVE_MOTOR_SPEED),
            () -> m_IO.motorOff(),
            this
        );
    }
    public Command manualMoveBackwardCommand() {
        return new StartEndCommand(
            () -> m_IO.setVoltage(-ArmConstants.MANUAL_MOVE_MOTOR_SPEED),
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
            (interrupted) -> {},
            () -> false,
            this
        );
    }

    // helper commands
    private void initalizeMoveTo(TrapezoidProfile.State goal) {
        m_controller.reset();
        m_goal = goal;
        m_profiledReference = new TrapezoidProfile.State(m_IO.getPosition(), m_IO.getVelocity());
        m_profile = new TrapezoidProfile(m_constraints);
    }

    private void executeMoveTo() {
        // recalculate the profiled reference point (the vel + pos that we want)
        m_profiledReference = m_profile.calculate(0.02, m_profiledReference, m_goal);
        
        // calculate part of the power based on target velocity 
        double feedForwardPower = m_feedforward.calculate(m_profiledReference.position - ArmConstants.POSITION_OFFSET, m_profiledReference.velocity);

        // calculate part of the power based on target position + current position
        double PIDPower = m_controller.calculate(m_IO.getPosition(), m_profiledReference.position);

        m_IO.setVoltage(feedForwardPower + PIDPower);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("arm/position", m_IO.getPosition());
        SmartDashboard.putNumber("arm/velocity", m_IO.getVelocity());
    }
}
