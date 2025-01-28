// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;


/** An example command that uses an example subsystem. */
public class AutoAlignCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final CommandSwerveDrivetrain m_drivetrain;
  private final VisionSubsystem m_vision;
  private Rotation2d m_targetRotation;
  private Translation2d m_targetTranslation;
  private Pose2d m_startPose;
  private boolean m_isTargetFound;
  private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle().withDriveRequestType(DriveRequestType.Velocity);
  private PIDController m_xController = new PIDController(DrivetrainConstants.ROBOT_POSITION_P, DrivetrainConstants.ROBOT_POSITION_I, DrivetrainConstants.ROBOT_POSITION_D); 
  private PIDController m_yController = new PIDController(DrivetrainConstants.ROBOT_POSITION_P, DrivetrainConstants.ROBOT_POSITION_I, DrivetrainConstants.ROBOT_POSITION_D); 

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoAlignCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision) {
    m_drivetrain = drivetrain;
    m_vision = vision;
    m_isTargetFound = false;

    driveFacingAngle.HeadingController = new PhoenixPIDController(DrivetrainConstants.ROBOT_ROTATION_P/10.0, DrivetrainConstants.ROBOT_ROTATION_I, DrivetrainConstants.ROBOT_ROTATION_D);
    driveFacingAngle.HeadingController.enableContinuousInput(0, Math.PI * 2);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // private Rotation2d getRotationToTag(){
  //   Translation2d translation = m_vision.getRobotTranslationToTag();
  //   if (translation.equals(Translation2d.kZero)){ // if tag not found, press trigger again
  //     return Rotation2d.kZero;
  //   }
  //   return m_vision.getRobotTranslationToTag().getAngle();
  // }
  
  /*
   * return empty (nothing) if no tag found,
   * else return robot to tag Rotation2d angle
   */
  private Optional<Rotation2d> getRotationToTag(){
    Translation2d translation = m_vision.getRobotTranslationToTag();
    if (translation.equals(Translation2d.kZero)){
      return Optional.empty();
    }
    return Optional.of(m_vision.getRobotTranslationToTag().getAngle());
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_isTargetFound = false;
    m_targetRotation = m_drivetrain.getRobotRotation();
    m_startPose = m_drivetrain.getState().Pose;
  }

  // Called every time the scheduler runs while the command is scheduled.
  /*
   * rotate drivetrain m_targetRotation deg
   */
  @Override
  public void execute() {      
    if (!m_isTargetFound){
      if (getRotationToTag().isPresent()){ // checks if value is empty (NOT ROBOT AT 0,0)
      // if (!getRotationToTag().equals(Rotation2d.kZero)){
        m_isTargetFound = true;
        m_targetRotation = m_drivetrain.getRobotRotation().minus(getRotationToTag().get());
        m_targetTranslation = m_vision.getRobotTranslationToTag();
      }
    }
    if (m_isTargetFound) {
      m_drivetrain.applyRequest(() -> driveFacingAngle
      .withTargetDirection(m_targetRotation)
      .withVelocityX(m_yController.calculate(m_drivetrain.getState().Pose.getX() - m_startPose.getX(), m_targetTranslation.getX()))
      .withVelocityY(m_xController.calculate(m_drivetrain.getState().Pose.getY() - m_startPose.getY(), m_targetTranslation.getY())));
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
