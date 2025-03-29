package frc.robot.subsystems.collar;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

import frc.robot.Constants.CollarConstants;
import frc.robot.RobotState;

public class CollarSubsystem extends SubsystemBase {
    private final CollarIOHardware m_IO = new CollarIOHardware();
    
    // constructor
    public CollarSubsystem() {
        m_IO.resetPosition();
    }

    public Command runCollarOff() {
        return new FunctionalCommand(
            () -> {m_IO.setVoltage(0);  SmartDashboard.putString("collar/currentlyRunningCommand", "runCollarOff");},
            () -> {m_IO.setVoltage(0);  SmartDashboard.putString("collar/currentlyRunningCommand", "runCollarOff");},
            (interrupted) -> {},
            () -> false,
            this
        ).withName("collarOff");
    }

    public Command runCollarOffInstant() {
        return new InstantCommand(
            () -> {m_IO.setVoltage(0); SmartDashboard.putString("collar/currentlyRunningCommand", "runCollarOffInstant");},
            this
        ).withTimeout(0).withName("collarOffInstant");
    }

    public Command runCollar(double speed) {
        return new FunctionalCommand(
            // set voltage in init also
            () -> {m_IO.setVoltage(speed); SmartDashboard.putString("collar/currentlyRunningCommand", "runCollar");},
            () -> {m_IO.setVoltage(speed); SmartDashboard.putString("collar/currentlyRunningCommand", "runCollar");},
            (interrupted) -> {},
            () -> false,
            this
        ).withName("runCollar");
    }

    public Command setVoltageToZeroCommand() {
        return new RunCommand(() -> m_IO.setVoltage(0), this).ignoringDisable(true);
    }

    public Command resetEncoders() {
        return new InstantCommand(() -> m_IO.setPosition(0), this);
    }

    @Override
    public void periodic() {
        SmartDashboard.putData("collar/subsystem", this);
        SmartDashboard.putNumber("collar/current", m_IO.getCurrent());
        SmartDashboard.putNumber("collar/velocity", m_IO.getVelocity());
        SmartDashboard.putNumber("collar/position", m_IO.getPosition());

    }

    public Command setCoastCommand() {
        return run(() -> m_IO.setCoast()).ignoringDisable(true);
    }
 }

