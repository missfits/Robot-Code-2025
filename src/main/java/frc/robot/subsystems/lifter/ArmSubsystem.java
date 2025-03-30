package frc.robot.subsystems.lifter;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.RobotStateConstants;

public class ArmSubsystem extends SubsystemBase {
    private final ArmIOHardware m_IO = new ArmIOHardware();
    private ArmFeedforward m_feedforward = new ArmFeedforward(
        ArmConstants.kS,
        ArmConstants.kG,
        ArmConstants.kV,
        ArmConstants.kA
    );
    private PIDController m_controller = new PIDController(
        ArmConstants.kP,
        ArmConstants.kI,
        ArmConstants.kD
    );
    private TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(
        ArmConstants.kMaxV, ArmConstants.kMaxA
    );
    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State(0,0);
    private TrapezoidProfile.State m_profiledReference;
    private TrapezoidProfile m_profile;

    private boolean runKeepInPlacePID = true; // false if a manual move command was just ran


    // constructor
    public ArmSubsystem() {
        m_IO.resetPosition(); 
        
        resetControllers();

        m_controller.enableContinuousInput(0, Math.PI*2);
    }

    // commands
    public Command keepInPlaceCommand() {
        return new RunCommand(
            () -> m_IO.setVoltage(ArmConstants.kG), 
            this
        ).withName("keepInPlace");
    }

    public Command keepInPlacePIDCommand() {
        return new RunCommand(
            () -> executeKeepInPlacePID(),
            this
        ).withName("keepInPlacePID");
    }

    public Command manualMoveCommand() {
        return new RunCommand(
            () -> {m_IO.setVoltage(ArmConstants.MANUAL_MOVE_MOTOR_SPEED); runKeepInPlacePID = false;},
            this
        ).withName("manualMoveCommand");
    }

    public Command manualMoveBackwardCommand() {
        return new RunCommand(
            () -> {m_IO.setVoltage(-ArmConstants.MANUAL_MOVE_MOTOR_SPEED); runKeepInPlacePID = false;},
            this
        ).withName("manualMoveBackwardCommand");
    }

    public Command manualMoveCommand(DoubleSupplier inputSupplier) {
        return new RunCommand(
            () -> {m_IO.setVoltage(inputSupplier.getAsDouble()); runKeepInPlacePID = false;},
            this
        ).withName("manualMoveCommand");
    }

    public Command moveToCommand(DoubleSupplier targetPositionSupplier) {
        return moveToCommand(() -> new TrapezoidProfile.State(targetPositionSupplier.getAsDouble(), 0));
    }

    public Command moveToCommand(Supplier<TrapezoidProfile.State> goal) {
        return new FunctionalCommand(
            () -> initalizeMoveTo(goal.get()),
            () -> executeMoveToOnMotor(),
            (interrupted) -> {},
            () -> false, 
            this
        ).withName("moveToCommand");
    }

    public Command moveToCommand(double targetPosition) {
        return moveToCommand(new TrapezoidProfile.State(targetPosition, 0));
    }

    public Command moveToCommand(TrapezoidProfile.State goal) {
        return new FunctionalCommand(
            () -> initalizeMoveTo(goal),
            () -> executeMoveToOnMotor(),
            (interrupted) -> {},
            () -> false,
            this
        ).withName("moveToCommand");
    }

    public Command setVoltageToZeroCommand() {
        return new RunCommand(() -> m_IO.setVoltage(0), this).ignoringDisable(true);
    }

    // helper commands
    private void initalizeMoveTo(TrapezoidProfile.State goal) {
        m_controller.reset();
        m_goal = goal;
        m_profiledReference = new TrapezoidProfile.State(m_IO.getPosition(), m_IO.getVelocity());
        m_profile = new TrapezoidProfile(m_constraints);
        runKeepInPlacePID = true;
    }

    private void executeMoveTo() {
        // recalculate the profiled reference point (the vel + pos that we want)
        m_profiledReference = m_profile.calculate(0.02, m_profiledReference, m_goal);
        
        // calculate part of the power based on target velocity 
        double feedForwardPower = m_feedforward.calculate(m_profiledReference.position - ArmConstants.POSITION_OFFSET, m_profiledReference.velocity);

        // calculate part of the power based on target position + current position
        double PIDPower = m_controller.calculate(m_IO.getPosition(), m_profiledReference.position);

        m_IO.setVoltage(feedForwardPower + PIDPower);

        SmartDashboard.putNumber("arm/target position", m_profiledReference.position);
        SmartDashboard.putNumber("arm/target velocity", m_profiledReference.velocity);

    }

    private void executeMoveToOnMotor() {
        // recalculate the profiled reference point (the vel + pos that we want)
        m_profiledReference = m_profile.calculate(0.02, m_profiledReference, m_goal);
        
        // calculate part of the power based on target velocity 
        double feedForwardPower = m_feedforward.calculate(m_profiledReference.position - ArmConstants.POSITION_OFFSET, m_profiledReference.velocity);
    
        m_IO.setClosedLoopPositionVoltage(m_profiledReference.position, feedForwardPower);;

        SmartDashboard.putNumber("arm/target position", m_IO.getTargetPosition());
        SmartDashboard.putNumber("arm/target velocity", m_profiledReference.velocity);

    }

    private void executeKeepInPlacePID() {

        if (runKeepInPlacePID) {

            // calculate part of the power based on target position + current position
            double PIDPower = m_controller.calculate(m_IO.getPosition(), m_goal.position);

            // calculate part of the power based on target velocity 
            double feedForwardPower = m_feedforward.calculate(m_IO.getPosition() - ArmConstants.POSITION_OFFSET, 0);

            m_IO.setVoltage(PIDPower + feedForwardPower);
        
            SmartDashboard.putNumber("arm/target position", m_goal.position);
            SmartDashboard.putNumber("arm/target velocity", 0);
            
        } else {
            m_IO.setVoltage(ArmConstants.kG);
        }
    }

    private void executeKeepInPlacePIDOnMotor() {

        if (runKeepInPlacePID) {

            // calculate part of the power based on target velocity 
            double feedForwardPower = m_feedforward.calculate(m_IO.getPosition() - ArmConstants.POSITION_OFFSET, 0);

            m_IO.setClosedLoopPositionVoltage(m_goal.position, feedForwardPower);

            SmartDashboard.putNumber("arm/target position", m_goal.position);
            SmartDashboard.putNumber("arm/target velocity", 0);
        
        } else {
            m_IO.setVoltage(ArmConstants.kG);
        }
    }

    public Trigger isAtGoal() {
        return new Trigger(() -> isAtPosition(m_goal.position));
    } 

    public Trigger isAtGoal(double goal) {
        return new Trigger(() -> isAtPosition(goal));
    } 

    private boolean isAtPosition(double goal) {
        return Math.abs(m_IO.getPosition() - goal) < ArmConstants.MAX_POSITION_TOLERANCE;
    } 
    
    public Trigger okToMoveElevatorDownTrigger() {
        return new Trigger(() -> okToMoveElevatorDown());
    } 

    private boolean okToMoveElevatorDown() {
        return  m_IO.getPosition() > ArmConstants.MIN_POS_ELEVATOR_CLEAR - ArmConstants.MAX_POSITION_TOLERANCE;
    } 

    public Trigger isArmInsideRobotTrigger() {
        return new Trigger(() -> isArmInsideRobot());
    }

    private boolean isArmInsideRobot() {
        return m_IO.getPosition() < ArmConstants.LOWER_INSIDE_ROBOT_BOUND 
            + ArmConstants.MAX_POSITION_TOLERANCE 
            && m_IO.getPosition() > ArmConstants.UPPER_INSIDE_ROBOT_BOUND
            - ArmConstants.MAX_POSITION_TOLERANCE;
    }

    public void resetControllers() {
        m_feedforward = new ArmFeedforward(
            ArmConstants.kS,
            ArmConstants.kG,
            ArmConstants.kV,
            ArmConstants.kA
        );
        m_controller = new PIDController(
            ArmConstants.kP,
            ArmConstants.kI,
            ArmConstants.kD
        );
            m_constraints = new TrapezoidProfile.Constraints(
            ArmConstants.kMaxV, ArmConstants.kMaxA
        );

        m_IO.resetSlot0Gains();
    } 
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("arm/position", m_IO.getPosition());
        SmartDashboard.putNumber("arm/velocity", m_IO.getVelocity());
        SmartDashboard.putNumber("arm/voltage", m_IO.getVoltage());
        SmartDashboard.putNumber("arm/goal position", m_goal.position);

        SmartDashboard.putData("arm/subsystem", this);
        SmartDashboard.putBoolean("arm/okToMoveElevatorDownTrigger", okToMoveElevatorDownTrigger().getAsBoolean());
        SmartDashboard.putBoolean("arm/isArmInsideRobotTrigger", isArmInsideRobotTrigger().getAsBoolean());

    }

    
    public void setBrake(boolean brake) {
       m_IO.setBrake(brake);
    }

    public void setRunKeepInPlace(boolean bool) {
        runKeepInPlacePID = bool;
    }

    public Command setCoastCommand() {
        return run(() -> m_IO.setCoast()).ignoringDisable(true);
    }
}
