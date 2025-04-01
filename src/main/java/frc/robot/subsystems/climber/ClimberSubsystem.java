package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.lifter.ArmIOHardware;

public class ClimberSubsystem extends SubsystemBase {
    private final ClimberIOHardware m_IO = new ClimberIOHardware();

    public ClimberSubsystem() {
        m_IO.setPosition(ClimberConstants.INITIAL_POSITION);
    }

    public Command runClimberOff() {
        return new RunCommand(
            () -> m_IO.setVoltage(0), 
            this
        );
    }

    public Command manualMoveCommand() {
        return new RunCommand(
            () -> runMotorInRightDirection(ClimberConstants.MANUAL_MOVE_MOTOR_SPEED),
            this
        );
    }

    public Command deployClimberCommand() {
        return new FunctionalCommand(
            () -> {},
            () -> runMotorInRightDirection(ClimberConstants.AUTO_MOVE_MOTOR_SPEED),
            (interrupted) -> m_IO.motorOff(),
            () -> (m_IO.getPosition() < ClimberConstants.DEPLOY_POSITION),
            this
        );
    }

    public Command liftClimberCommand() {
        return new FunctionalCommand(
            () -> {},
            () -> runMotorInRightDirection(ClimberConstants.AUTO_MOVE_MOTOR_SPEED),
            (interrupted) -> m_IO.motorOff(),
            () -> (m_IO.getPosition() < ClimberConstants.LIFT_POSITION),
            this
        );
    }

    public void setEnabledNeutralMode() {
        m_IO.setBrake(true);
    }

    public void setDisabledNeutralMode() {
        m_IO.setBrake(false);
    }

    public Command setCoastCommand() {
        return run(() -> m_IO.setCoast()).ignoringDisable(true);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("climber/position", m_IO.getPosition());
        SmartDashboard.putNumber("climber/current", m_IO.getCurrent());
    }

    private void runMotorInRightDirection(double speed) {
        m_IO.setVoltage(-MathUtil.clamp(speed, 0, 20));
    }

}
