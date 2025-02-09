// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class RotateToFaceReefCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final CommandSwerveDrivetrain m_drivetrain;
  private final VisionSubsystem m_vision;
  private Rotation2d targetRotation;
  private  boolean targetFound = false;;
  private final SwerveRequest.FieldCentricFacingAngle snapToAngle = new SwerveRequest.FieldCentricFacingAngle().withDriveRequestType(DriveRequestType.Velocity).withVelocityX(0).withVelocityY(0);
  
  
    /**
     * Creates a new RotateToFaceReefCommand, which rotates the robot parallel to a seen AprilTag
     *
     * @param drivetrain The drivetrain subsystem used by this command.
     * @param vision The vision subsystem used by this command.
     */
    public RotateToFaceReefCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision) {
      m_drivetrain = drivetrain;
      m_vision = vision;
  
      snapToAngle.HeadingController = new PhoenixPIDController(DrivetrainConstants.ROBOT_ROTATION_P, DrivetrainConstants.ROBOT_ROTATION_I, DrivetrainConstants.ROBOT_ROTATION_D);
      snapToAngle.HeadingController.enableContinuousInput(0, Math.PI * 2);
  
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(drivetrain);
    }
    
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
  
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {      
    // only get targetRotation once
    if (m_vision.getTargetFound() && !targetFound) {
      targetRotation = Rotation2d.fromRadians(Math.PI + m_vision.getTargetYaw());
      targetFound = true;
    }

    // only rotate drivetrain if targetRotation is found
    if (targetFound) {
      m_drivetrain.setControl(snapToAngle.withTargetDirection(targetRotation));
    }
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
