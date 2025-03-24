// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

public class DriveToReefCommand extends Command {

  public enum ReefPosition {
    LEFT, RIGHT
  }

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final CommandSwerveDrivetrain m_drivetrain;
  private Translation2d m_targetTranslation = new Translation2d();
  private Rotation2d targetRotation;
  private final VisionSubsystem m_vision;
  private final ReefPosition m_side;
  private final LEDSubsystem m_ledSubsystem;

  private final ProfiledPIDController xController = new ProfiledPIDController(DrivetrainConstants.ROBOT_POSITION_P, DrivetrainConstants.ROBOT_POSITION_I, DrivetrainConstants.ROBOT_POSITION_D, new TrapezoidProfile.Constraints(AutoAlignConstants.kMaxV, AutoAlignConstants.kMaxA));
  private final ProfiledPIDController yController = new ProfiledPIDController(DrivetrainConstants.ROBOT_POSITION_P, DrivetrainConstants.ROBOT_POSITION_I, DrivetrainConstants.ROBOT_POSITION_D, new TrapezoidProfile.Constraints(AutoAlignConstants.kMaxV, AutoAlignConstants.kMaxA));
  private final SwerveRequest.FieldCentricFacingAngle driveRequest = new SwerveRequest.FieldCentricFacingAngle().withDriveRequestType(DriveRequestType.Velocity).withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);
  
  
    /**
     * Creates a new DriveToReefCommand, which moves the robot to a seen AprilTag AFTER rotating to face parallel
     *
     * @param drivetrain The drivetrain subsystem used by this command.
     * @param Pose2d The target pose (but only Translation) used by this command.
     */
    public DriveToReefCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision, ReefPosition side, LEDSubsystem ledSubsystem) {
      m_drivetrain = drivetrain;
      m_vision = vision;
      m_side = side;
      m_ledSubsystem = ledSubsystem;
    
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(drivetrain);
    }
    
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {


      Pose2d targetPose = m_vision.getClosestReefAprilTag(m_drivetrain.getState().Pose);
      if (targetPose != null) {
        // offset with robot size because the robot has width and is not a point!
        // see images/drive to reef with offset.png for right/left offset math
        if (m_side.equals(ReefPosition.RIGHT)) {
          // for the right side add reef offset value
          m_targetTranslation = new Translation2d(
            targetPose.getX()
            + DrivetrainConstants.ROBOT_SIZE_X/2 * Math.cos(targetPose.getRotation().getRadians())
            + AutoAlignConstants.REEF_OFFSET_RIGHT * Math.cos(Math.PI/2 + targetPose.getRotation().getRadians()), 
            targetPose.getY()
            + DrivetrainConstants.ROBOT_SIZE_X/2 * Math.sin(targetPose.getRotation().getRadians())
            + AutoAlignConstants.REEF_OFFSET_RIGHT * Math.sin(Math.PI/2 + targetPose.getRotation().getRadians()));
        } else {
          // for the left side subtract reef offset value
          m_targetTranslation = new Translation2d(
            targetPose.getX()
            + DrivetrainConstants.ROBOT_SIZE_X/2 * Math.cos(targetPose.getRotation().getRadians())
            - AutoAlignConstants.REEF_OFFSET_LEFT * Math.cos(Math.PI/2 + targetPose.getRotation().getRadians()), 
            targetPose.getY()
            + DrivetrainConstants.ROBOT_SIZE_X/2 * Math.sin(targetPose.getRotation().getRadians())
            - AutoAlignConstants.REEF_OFFSET_LEFT * Math.sin(Math.PI/2 + targetPose.getRotation().getRadians()));
        }
        
    
      } else {
        m_targetTranslation = m_drivetrain.getState().Pose.getTranslation();
      }

      targetRotation = targetPose.getRotation().plus(Rotation2d.fromRadians(Math.PI));

      xController.reset(m_drivetrain.getState().Pose.getX());
      yController.reset(m_drivetrain.getState().Pose.getY());

      driveRequest.HeadingController = new PhoenixPIDController(DrivetrainConstants.ROBOT_ROTATION_P, DrivetrainConstants.ROBOT_ROTATION_I, DrivetrainConstants.ROBOT_ROTATION_D);
      driveRequest.HeadingController.enableContinuousInput(0, Math.PI * 2);
      
      SmartDashboard.putString("drivetoreef/target robot pose", m_targetTranslation.toString());

    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

      m_drivetrain.setisAutoAlign(isAligned());

      double xVelocity = xController.calculate(m_drivetrain.getState().Pose.getX(), m_targetTranslation.getX()) + xController.getSetpoint().velocity;
      double yVelocity = yController.calculate(m_drivetrain.getState().Pose.getY(), m_targetTranslation.getY()) + yController.getSetpoint().velocity;

      m_drivetrain.setControl(driveRequest
        .withVelocityX(xVelocity) 
        .withVelocityY(yVelocity) 
        .withTargetDirection(targetRotation)
      );

      SmartDashboard.putNumber("drivetoreef/setpoint position x", xController.getSetpoint().position);
      SmartDashboard.putNumber("drivetoreef/setpoint position y", yController.getSetpoint().position);
      
      SmartDashboard.putNumber("drivetoreef/setpoint velocity x", xController.getSetpoint().velocity);
      SmartDashboard.putNumber("drivetoreef/setpoint velocity y", yController.getSetpoint().velocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.setisAutoAlign(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public boolean isAligned(){
    Pose2d drivetrainPose = m_drivetrain.getState().Pose;
    // change to absolute value?
    double xDist = Math.abs(drivetrainPose.getX() - m_targetTranslation.getX());
    double yDist = Math.abs(drivetrainPose.getY() - m_targetTranslation.getY());
    SmartDashboard.putNumber("drivetrain/auto-alignment-xdist", xDist);
    SmartDashboard.putNumber("drivetrain/auto-alignment-ydist", yDist);
    return ((xDist < VisionConstants.VISION_ALIGNMENT_DISCARD) && (yDist < VisionConstants.VISION_ALIGNMENT_DISCARD));
  }
}