// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** A command that drives forward 1m ROBOT CENTRIC. */
public class Drive1mCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final CommandSwerveDrivetrain m_drivetrain;
  private Pose2d m_startPose;

  private PIDController xController = new PIDController(DrivetrainConstants.ROBOT_POSITION_P, DrivetrainConstants.ROBOT_POSITION_I, DrivetrainConstants.ROBOT_POSITION_D);
  private PIDController yController = new PIDController(DrivetrainConstants.ROBOT_POSITION_P, DrivetrainConstants.ROBOT_POSITION_I, DrivetrainConstants.ROBOT_POSITION_D);

  private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity).withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

  /**
   * Creates a new Drive5mCommand.
   *s
   * @param subsystem The subsystem used by this command.
   */
  public Drive1mCommand(CommandSwerveDrivetrain drivetrain) {
    m_drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // reset controllers to adjustable PID values through elastic 
    xController = new PIDController(
        SmartDashboard.getNumber("drivetrain constants/kP", DrivetrainConstants.ROBOT_POSITION_P), 
        SmartDashboard.getNumber("drivetrain constants/kI", DrivetrainConstants.ROBOT_POSITION_I),
        SmartDashboard.getNumber("drivetrain constants/kD", DrivetrainConstants.ROBOT_POSITION_D)
    );

    yController = new PIDController(
        SmartDashboard.getNumber("drivetrain constants/kP", DrivetrainConstants.ROBOT_POSITION_P), 
        SmartDashboard.getNumber("drivetrain constants/kI", DrivetrainConstants.ROBOT_POSITION_I),
        SmartDashboard.getNumber("drivetrain constants/kD", DrivetrainConstants.ROBOT_POSITION_D)
    );


    m_startPose = m_drivetrain.getState().Pose;
    xController.setSetpoint(m_startPose.getX() + 0.5);
    yController.setSetpoint(m_startPose.getY());


    SmartDashboard.putNumber("drive5m/setpoint position x", xController.getSetpoint());
    SmartDashboard.putNumber("drive5m/setpoint position y", yController.getSetpoint());

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_drivetrain.setControl(driveRequest
        .withVelocityX(xController.calculate(m_drivetrain.getState().Pose.getX())) 
        .withVelocityY(0) 
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
