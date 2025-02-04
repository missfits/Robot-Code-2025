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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AutoAlignCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final CommandSwerveDrivetrain m_drivetrain;
  private final VisionSubsystem m_vision;
  private Rotation2d m_targetRotation;
  private boolean m_isTargetFound;
  private final SwerveRequest.FieldCentricFacingAngle snapToAngle = new SwerveRequest.FieldCentricFacingAngle().withDriveRequestType(DriveRequestType.Velocity).withVelocityX(0).withVelocityY(0);


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoAlignCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision) {
    m_drivetrain = drivetrain;
    m_vision = vision;
    m_isTargetFound = false;

    snapToAngle.HeadingController = new PhoenixPIDController(DrivetrainConstants.ROBOT_ROTATION_P, DrivetrainConstants.ROBOT_ROTATION_I, DrivetrainConstants.ROBOT_ROTATION_D);
    snapToAngle.HeadingController.enableContinuousInput(0, Math.PI * 2);

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
  }

  // Called every time the scheduler runs while the command is scheduled.
  /*
   * rotate drivetrain m_targetRotation deg
   */
  @Override
  public void execute() {      
    if (!m_isTargetFound){ // only set target rotation once
      if (getRotationToTag().isPresent()){ // checks if value is empty (NOT ROBOT AT 0,0)
        m_isTargetFound = true;
        m_targetRotation = m_drivetrain.getRobotRotation().minus(getRotationToTag().get());
      }
    }

    if (getRotationToTag().isPresent()){
      SmartDashboard.putNumber("autoaligncommand/rotationtotag", getRotationToTag().get().getDegrees());
      SmartDashboard.putNumber("autoaligncommand/targetrotation", m_targetRotation.getDegrees());
      SmartDashboard.putNumber("autoaligncommand/robotrotation", m_drivetrain.getRobotRotation().getDegrees());
    }

    m_drivetrain.setControl(snapToAngle.withTargetDirection(m_targetRotation));
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
