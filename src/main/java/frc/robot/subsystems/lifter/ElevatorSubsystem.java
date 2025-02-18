package frc.robot.subsystems.lifter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
    public ElevatorSubsystem() {
        m_IO.resetPosition();
    }

    // commands
    public Command keepInPlaceCommand() {
        return new RunCommand(
            () -> m_IO.setVoltage(ElevatorConstants.kG),
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
        double feedForwardPower = m_feedforward.calculate(m_profiledReference.velocity);

        // calculate part of the power based on target position + current position
        double PIDPower = m_controller.calculate(m_IO.getPosition(), m_profiledReference.position);

        m_IO.setVoltage(feedForwardPower + PIDPower);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("elevator/position", m_IO.getPosition());
        SmartDashboard.putNumber("elevator/velocity", m_IO.getVelocity());
    }

    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutDistance m_distance = Meters.mutable(0);
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);


    private final SysIdRoutine m_sysIDRoutineElevatorRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.per(Seconds).of(0.5), Volts.of(3), Seconds.of(10)),
        new SysIdRoutine.Mechanism(
            output -> m_IO.setVoltage(output.in(Volts)), 
            log -> {
                // Record a frame for the shooter motor.
                log.motor("elevatorsysID")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            m_IO.getVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(m_IO.getPosition(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(m_IO.getVelocity(), MetersPerSecond));
              },
            this
        )
    );

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIDRoutineElevatorRoutine.quasistatic(direction);
      }
      
      public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIDRoutineElevatorRoutine.dynamic(direction);
      }
}
