package frc.robot.subsystems.lifter;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
    
    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State(0,0);

    private boolean runKeepInPlacePID = true; // false if a manual move command was just ran


    // constructor
    public ArmSubsystem() {
        m_IO.resetPosition();

        resetControllers();
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
            () -> executeMoveTo(),
            (interrupted) -> {},
            () -> Math.abs(m_IO.getPosition()-goal.get().position) < 0.025, // equivalent to 1 degree
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
        m_goal = goal;
        runKeepInPlacePID = true;
    }

    private void executeMoveTo() {
        m_IO.setControl(new MotionMagicVoltage(m_goal.position/ArmConstants.DEGREES_PER_ROTATION).withFeedForward(ArmConstants.kG).withSlot(0));
    }

    private void executeKeepInPlacePID() {

        if (runKeepInPlacePID) {
            m_IO.setControl(new MotionMagicVoltage(m_goal.position/ArmConstants.DEGREES_PER_ROTATION).withFeedForward(ArmConstants.kG).withSlot(0));
        } else {
            m_IO.setVoltage(ArmConstants.kG);
        }
    }

    private boolean isAtPosition(double goal) {
        return Math.abs(m_IO.getPosition() - goal) < ArmConstants.MAX_POSITION_TOLERANCE;
    } 

    public Trigger isAtGoal() {
        return new Trigger(() -> isAtPosition(m_goal.position));
    } 

    public Trigger isAtGoal(double goal) {
        return new Trigger(() -> isAtPosition(goal));
    } 
    
    public Trigger okToMoveElevatorDownTrigger() {
        return new Trigger(() -> okToMoveElevatorDown());
    } 

    private boolean okToMoveElevatorDown() {
        return  m_IO.getPosition() > ArmConstants.MIN_POS_ELEVATOR_CLEAR;
    } 

    public void resetControllers() {
        var talonFXConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = ArmConstants.kS/ArmConstants.DEGREES_PER_ROTATION; 
        slot0Configs.kV = ArmConstants.kV/ArmConstants.DEGREES_PER_ROTATION; 
        slot0Configs.kA = ArmConstants.kA/ArmConstants.DEGREES_PER_ROTATION; 
        slot0Configs.kP = ArmConstants.kP/ArmConstants.DEGREES_PER_ROTATION;
        slot0Configs.kI = ArmConstants.kI/ArmConstants.DEGREES_PER_ROTATION;
        slot0Configs.kD = ArmConstants.kD/ArmConstants.DEGREES_PER_ROTATION;

        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ArmConstants.kMaxV/ArmConstants.DEGREES_PER_ROTATION; 
        motionMagicConfigs.MotionMagicAcceleration = ArmConstants.kMaxA/ArmConstants.DEGREES_PER_ROTATION; 
        motionMagicConfigs.MotionMagicJerk = 0; // no jerk limit

        m_IO.config(talonFXConfigs);
    } 
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("arm/position", m_IO.getPosition());
        SmartDashboard.putNumber("arm/velocity", m_IO.getVelocity());
        SmartDashboard.putNumber("arm/voltage", m_IO.getVoltage());

        SmartDashboard.putNumber("arm/target position", m_IO.getTargetPosition());
        SmartDashboard.putNumber("arm/target velocity", m_IO.getTargetVelocity());


        SmartDashboard.putData("arm/subsystem", this);
        SmartDashboard.putBoolean("arm/okToMoveElevatorDownTrigger", okToMoveElevatorDownTrigger().getAsBoolean());

    }

    
    public void setBrake(boolean brake) {
       m_IO.setBrake(brake);
    }

    public void setRunKeepInPlace(boolean bool) {
        runKeepInPlacePID = bool;
    }
}
