package frc.robot.subsystems.lifter;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.RobotStateConstants;

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

    private boolean runKeepInPlacePID = true; // false if a manual move command was just ran

    // constructor
    public ElevatorSubsystem() {
        m_IO.resetPosition();
        resetControllers();
    }

    // commands
    public Command keepInPlaceCommand() {
        return new RunCommand(
            () -> m_IO.setVoltage(ElevatorConstants.kG),
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
            () -> {m_IO.setVoltage(ElevatorConstants.MANUAL_MOVE_MOTOR_SPEED); runKeepInPlacePID = false;},
            this
        ).withName("manualMoveCommand");
    }

    public Command manualMoveBackwardCommand() {
        return new RunCommand(
            () -> {m_IO.setVoltage(-ElevatorConstants.MANUAL_MOVE_MOTOR_SPEED); runKeepInPlacePID = false;},
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
            () -> executeMoveTo(),
            (interrupted) -> {},
            () -> Math.abs(m_IO.getPosition()-goal.get().position) < 0.005,
            this
        ).withName("moveToCommand");
    }

    public Command moveToCommand(double targetPosition) {
        return moveToCommand(new TrapezoidProfile.State(targetPosition, 0));
    }

    public Command moveToCommand(TrapezoidProfile.State goal) {
        return new FunctionalCommand(
            () -> initalizeMoveTo(goal),
            () -> executeMoveTo(),
            (interrupted) -> {SmartDashboard.putBoolean("elevator/moveToCommandRunning", false);},
            () -> false,
            this
        ).withName("moveToCommand");
    }

    public Command moveToOnMotorCommand(double targetPosition) {
        return moveToOnMotorCommand(new TrapezoidProfile.State(targetPosition, 0));
    }

    public Command moveToOnMotorCommand(TrapezoidProfile.State goal) {
        return new FunctionalCommand(
            () -> initalizeMoveTo(goal),
            () -> executeMoveToOnMotor(),
            (interrupted) -> {},
            () -> false,
            this
        ).withName("moveToOnMotorCommand");
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
        double feedForwardPower = m_feedforward.calculate(m_profiledReference.velocity);

        // calculate part of the power based on target position + current position
        double PIDPower = m_controller.calculate(m_IO.getPosition(), m_profiledReference.position);

        m_IO.setVoltage(feedForwardPower + PIDPower);

        SmartDashboard.putNumber("elevator/target position", m_profiledReference.position);
        SmartDashboard.putNumber("elevator/target velocity", m_profiledReference.velocity);
        SmartDashboard.putBoolean("elevator/moveToCommandRunning", true);
    }

    private void executeMoveToOnMotor() {
        // recalculate the profiled reference point (the vel + pos that we want)
        m_profiledReference = m_profile.calculate(0.02, m_profiledReference, m_goal);
        
        // calculate part of the power based on target velocity 
        double feedForwardPower = m_feedforward.calculate(m_profiledReference.velocity);

        m_IO.setControl(new PositionVoltage(m_goal.position).withFeedForward(feedForwardPower).withSlot(0));

        SmartDashboard.putNumber("elevator/target position", m_IO.getTargetPosition());
        SmartDashboard.putNumber("elevator/target velocity", m_profiledReference.velocity);
    }

    private void executeKeepInPlacePID() {

        if (runKeepInPlacePID) {
            // calculate part of the power based on target position + current position
            double PIDPower = m_controller.calculate(m_IO.getPosition(), m_goal.position);

            // calculate part of the power based on target velocity 
            double feedForwardPower = m_feedforward.calculate(0);

            m_IO.setVoltage(PIDPower + feedForwardPower);
            SmartDashboard.putNumber("elevator/target position", m_goal.position);
            SmartDashboard.putNumber("elevator/target velocity", 0);
        
        } else {
            m_IO.setVoltage(ElevatorConstants.kG);
        }
    }


    private void executeKeepInPlacePIDOnMotor() {

        if (runKeepInPlacePID) {

            // calculate part of the power based on target velocity 
            double feedForwardPower = m_feedforward.calculate(0);

            m_IO.setControl(new PositionVoltage(m_goal.position).withFeedForward(feedForwardPower).withSlot(0));
    
            SmartDashboard.putNumber("elevator/target position", m_goal.position);
            SmartDashboard.putNumber("elevator/target velocity", 0);
        
        } else {
            m_IO.setVoltage(ElevatorConstants.kG);
        }
    }

    private boolean isAtPosition(double goal) {
        return Math.abs(m_IO.getPosition() - goal) < ElevatorConstants.MAX_POSITION_TOLERANCE;
    } 

    public Trigger isAtGoal() {
        return new Trigger(() -> isAtPosition(m_goal.position));
    } 
    
    public Trigger isAtGoal(double goal) {
        return new Trigger(() -> isAtPosition(goal));
    } 

    public Trigger okToMoveArmBackTrigger() {
        return new Trigger(() -> okToMoveArmBack());
    } 

    private boolean okToMoveArmBack() {
        return m_IO.getPosition() > ElevatorConstants.MIN_POS_ARM_CLEAR;
    } 
    
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

        var talonFXConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = ElevatorConstants.kS*ElevatorConstants.METERS_PER_ROTATION; 
        slot0Configs.kV = ElevatorConstants.kV*ElevatorConstants.METERS_PER_ROTATION; 
        slot0Configs.kA = ElevatorConstants.kA*ElevatorConstants.METERS_PER_ROTATION; 
        slot0Configs.kP = ElevatorConstants.kP*ElevatorConstants.METERS_PER_ROTATION;
        slot0Configs.kI = ElevatorConstants.kI*ElevatorConstants.METERS_PER_ROTATION;
        slot0Configs.kD = ElevatorConstants.kD*ElevatorConstants.METERS_PER_ROTATION;

        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ElevatorConstants.kMaxV*ElevatorConstants.METERS_PER_ROTATION; 
        motionMagicConfigs.MotionMagicAcceleration = ElevatorConstants.kMaxA*ElevatorConstants.METERS_PER_ROTATION; 
        motionMagicConfigs.MotionMagicJerk = 0; // no jerk limit

        m_IO.config(talonFXConfigs);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("elevator/position", m_IO.getPosition());
        SmartDashboard.putNumber("elevator/velocity", m_IO.getVelocity());
        SmartDashboard.putNumber("elevator/voltage", m_IO.getVoltage());

        SmartDashboard.putData("elevator/subsystem", this);
    }

    public void setBrake(boolean brake) {
        m_IO.setBrake(brake);
     }

    public void setRunKeepInPlace(boolean bool) {
        runKeepInPlacePID = bool;
    }

    public Trigger isNotAtL4Trigger() {
        return new Trigger(() -> isNotAtL4());
    }

    public boolean isNotAtL4() {
        return m_IO.getPosition() < ElevatorConstants.MIN_HEIGHT_TO_BE_BELOW_L4;
    }

    public Trigger isTallTrigger() {
       return new Trigger(() -> isTall());
    }

    public boolean isTall() {
        return m_IO.getPosition() > ElevatorConstants.MIN_HEIGHT_TO_BE_TALL;
    }
}
