package frc.robot.subsystems.lifter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase{
    private final ElevatorIOHardware m_IO = new ElevatorIOHardware();
    private ElevatorFeedforward m_feedforward = new ElevatorFeedforward(
        ElevatorConstants.kS,
        ElevatorConstants.kG,
        ElevatorConstants.kV,
        ElevatorConstants.kA
    );
    private PIDController m_controller = new PIDController(
        ElevatorConstants.kP,
        ElevatorConstants.kI,
        ElevatorConstants.kD
    );
    private TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(
        ElevatorConstants.kMaxV, ElevatorConstants.kMaxA
    );
    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State(0,0);
    private TrapezoidProfile.State m_profiledReference;
    private TrapezoidProfile m_profile;

    // constructor
    public ElevatorSubsystem() {
        m_IO.resetPosition();
    }

    // commands
    // public Command keepInPlaceCommand() {
    //     return new RunCommand(
    //         () -> m_IO.setVoltage(ElevatorConstants.kG),
    //         this
    //     );
    // }

    public Command keepInPlaceCommand() {
        return new FunctionalCommand(
            () -> {}, 
            () -> m_IO.setVoltage(ElevatorConstants.kG),
            (interrupted) -> {},
            () -> !isAtPosition(m_goal.position),
            this
        );
    }

    public Command manualMoveCommand() {
        return new RunCommand(
            () -> m_IO.setVoltage(ElevatorConstants.MANUAL_MOVE_MOTOR_SPEED),
            this
        );
    }

    public Command manualMoveBackwardCommand() {
        return new RunCommand(
            () -> m_IO.setVoltage(-ElevatorConstants.MANUAL_MOVE_MOTOR_SPEED),
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
            (interrupted) -> CommandScheduler.getInstance().schedule(keepInPlaceCommand()),
            () -> isAtPosition(goal.position),
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
        double feedForwardPower = m_feedforward.calculate(m_profiledReference.velocity);

        // calculate part of the power based on target position + current position
        double PIDPower = m_controller.calculate(m_IO.getPosition(), m_profiledReference.position);

        m_IO.setVoltage(feedForwardPower + PIDPower);

        SmartDashboard.putNumber("elevator/target position", m_profiledReference.position);
        SmartDashboard.putNumber("elevator/target velocity", m_profiledReference.velocity);
    }

    private boolean isAtPosition(double goal) {
        return Math.abs(m_IO.getPosition() - goal) < ElevatorConstants.MAX_POSITION_TOLERANCE;
    } 

    // public Trigger atPositionTrigger() {
    //     return new Trigger(() -> isAtPosition(m_goal.position));
    // }
    
    public void resetControllers() {
        m_feedforward = new ElevatorFeedforward(
            ElevatorConstants.kS,
            ElevatorConstants.kG,
            ElevatorConstants.kV,
            ElevatorConstants.kA
        );
        m_controller = new PIDController(
            ElevatorConstants.kP,
            ElevatorConstants.kI,
            ElevatorConstants.kD
        );
        m_constraints = new TrapezoidProfile.Constraints(
            ElevatorConstants.kMaxV, ElevatorConstants.kMaxA
        );
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("elevator/position", m_IO.getPosition());
        SmartDashboard.putNumber("elevator/velocity", m_IO.getVelocity());
    }

    public void setBrake(boolean brake) {
        m_IO.setBrake(brake);
     }
}
