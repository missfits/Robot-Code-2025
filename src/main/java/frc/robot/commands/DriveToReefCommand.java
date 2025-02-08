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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.wpilibj2.command.Command;

public class DriveToReefCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final CommandSwerveDrivetrain m_drivetrain;
  private final Translation2d m_targetTranslation;
  private final PIDController xController = new PIDController(DrivetrainConstants.ROBOT_POSITION_P, DrivetrainConstants.ROBOT_POSITION_I, DrivetrainConstants.ROBOT_POSITION_D);
  private final PIDController yController = new PIDController(DrivetrainConstants.ROBOT_POSITION_P, DrivetrainConstants.ROBOT_POSITION_I, DrivetrainConstants.ROBOT_POSITION_D);
  private final SwerveRequest.ApplyFieldSpeeds driveRequest = new SwerveRequest.ApplyFieldSpeeds().withDriveRequestType(DriveRequestType.Velocity);
  
  
    /**
     * Creates a new DriveToReefCommand, which moves the robot to a seen AprilTag AFTER rotating to face parallel
     *
     * @param drivetrain The drivetrain subsystem used by this command.
     * @param Pose2d The target pose (but only Translation) used by this command.
     */
    public DriveToReefCommand(CommandSwerveDrivetrain drivetrain, Pose2d targetPose) {
      m_drivetrain = drivetrain;
      
      // cos and sin offset because the robot has width and is not a point!
      m_targetTranslation = new Translation2d(
        targetPose.getX() - DrivetrainConstants.ROBOT_SIZE_X/2 * Math.cos(targetPose.getRotation().getRadians()), 
        targetPose.getY() - DrivetrainConstants.ROBOT_SIZE_X/2 * Math.sin(targetPose.getRotation().getRadians()));
    
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
      m_drivetrain.setControl(driveRequest
        .withSpeeds(new ChassisSpeeds(
          xController.calculate(m_drivetrain.getState().Pose.getX(), m_targetTranslation.getX()),
          yController.calculate(m_drivetrain.getState().Pose.getY(), m_targetTranslation.getY()),
          0)));
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
