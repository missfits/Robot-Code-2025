package frc.robot.subsystems.lifter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotContainer.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.LifterConstants;


public class LifterSubsystem extends SubsystemBase {
    private final ElevatorIOHardware m_elevatorIO = new ElevatorIOHardware();
    private final ArmIOHardware m_armIO = new ArmIOHardware();


    // constructor
    public LifterSubsystem() {

    }

    public Command getCommand(RobotState targetRobotState) {
        return new WaitCommand(0);
    }
}
