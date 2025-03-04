package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.lifter.ArmIOHardware;

public class ClimberSubsystem extends SubsystemBase {
    private final ClimberIOHardware m_IO = new ClimberIOHardware();

    public Command runClimberOff() {
        return new RunCommand(
            () -> m_IO.setVoltage(0), 
            this
        );
    }

    public Command manualMoveCommand() {
        return new RunCommand(
            () -> m_IO.setVoltage(ClimberConstants.MANUAL_MOVE_MOTOR_SPEED),
            this
        );
    }
    public Command manualMoveBackwardCommand() {
        return new RunCommand(
            () -> m_IO.setVoltage(-ClimberConstants.MANUAL_MOVE_MOTOR_SPEED),
            this
        );
    }

    public void setEnabledNeutralMode() {
        m_IO.setBrake(true);
    }

    public void setDisabledNeutralMode() {
        m_IO.setBrake(false);
    }

}
