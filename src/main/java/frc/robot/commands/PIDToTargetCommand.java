// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** A command that drives forward 1m ROBOT CENTRIC. */
public class PIDToTargetCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final CommandSwerveDrivetrain m_drivetrain;
  private Supplier<Pose2d> m_poseSupplier;
  private Pose2d m_targetPose;

  private PIDController xController = new PIDController(DrivetrainConstants.ROBOT_POSITION_P, DrivetrainConstants.ROBOT_POSITION_I, DrivetrainConstants.ROBOT_POSITION_D);
  private PIDController yController = new PIDController(DrivetrainConstants.ROBOT_POSITION_P, DrivetrainConstants.ROBOT_POSITION_I, DrivetrainConstants.ROBOT_POSITION_D);

  private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity).withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

  /**
   * Creates a new Drive5mCommand.
   *s
   * @param subsystem The subsystem used by this command.
   */
  public PIDToTargetCommand(CommandSwerveDrivetrain drivetrain, Supplier<Pose2d> poseSupplier) {
    m_drivetrain = drivetrain;
    m_poseSupplier = poseSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // reset controllers to adjustable PID values through elastic 
    xController = new PIDController(
        DrivetrainConstants.TO_TARGET_POSITION_P, 
        DrivetrainConstants.TO_TARGET_POSITION_I,
        DrivetrainConstants.TO_TARGET_POSITION_D
    );

    yController = new PIDController(
        DrivetrainConstants.TO_TARGET_POSITION_P, 
        DrivetrainConstants.TO_TARGET_POSITION_I,
        DrivetrainConstants.TO_TARGET_POSITION_D
    );


    m_targetPose = m_poseSupplier.get();
    m_targetPose = m_targetPose == null ? m_drivetrain.getState().Pose : m_targetPose;
    xController.setSetpoint(m_targetPose.getX());
    yController.setSetpoint(m_targetPose.getY());


    SmartDashboard.putNumber("drive5m/setpoint position x", xController.getSetpoint());
    SmartDashboard.putNumber("drive5m/setpoint position y", yController.getSetpoint());

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_drivetrain.setControl(driveRequest
        .withVelocityX(xController.calculate(m_drivetrain.getState().Pose.getX())) 
        .withVelocityY(yController.calculate(m_drivetrain.getState().Pose.getY())) 
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
