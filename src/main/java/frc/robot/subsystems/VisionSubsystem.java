// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.hardware.Pigeon2;

import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;



public class VisionSubsystem extends SubsystemBase {
  private final PhotonCamera m_camera = new PhotonCamera(VisionConstants.CAMERA_NAME);
  private final Pigeon2 m_gyro;
  // private final LEDSubsystem m_ledSubsystem;
  private Translation2d targetTranslation2d = new Translation2d(0,0); // distance to the target; updated every periodic() call if target is found 
  private boolean targetFound; // true if the translation2d was updated last periodic() call
  private Pose2d targetPose;
  private double targetYaw; // in radians, relative to field
  private Pose2d currentPose; // current robot pose, updates periodically
  private EstimatedRobotPose estimatedRobotPose; 

  private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  private PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.ROBOT_TO_CAM_3D);


  private double distToTargetX;
  private double distToTargetY;

  /** Creates a new Vision Subsystem. */
  public VisionSubsystem(Pigeon2 pigeon) {
    m_gyro = pigeon;
    
    distToTargetX = 1;
    distToTargetY = 1;    

  }

  public Translation2d getRobotTranslationToTag() {
    return targetFound ? targetTranslation2d.plus(VisionConstants.ROBOT_TO_CAM) : new Translation2d(0,0); // only return translation2d if target was found
  }


  /** returns yaw (rotation on field) of target in radians */
  public double getTargetYaw() {
    return targetYaw;
  }

  /** returns pose  of target in radians */
  public Pose2d getTargetPose() {
    return targetPose;
  }

  public boolean getTargetFound() {
    return targetFound;
  }

  public EstimatedRobotPose getEstimatedRobotPose() {
    return estimatedRobotPose;
  }


  @Override
  public void periodic() {

    double targetDistanceMeters = 0.0;

    var results = m_camera.getAllUnreadResults();

    if (!results.isEmpty()) {
      // Camera processed a new frame since last
      // Get the last one in the list.
      var result = results.get(results.size() - 1);

      // update the estimated robot pose if the pose estimator outputs something
      Optional<EstimatedRobotPose> poseEstimatorOutput = poseEstimator.update(result);
      if (poseEstimatorOutput.isPresent()) {
        estimatedRobotPose = poseEstimatorOutput.get(); 
      }

      if (result.hasTargets()) {
        PhotonTrackedTarget target = result.getBestTarget();
        double biggestTargetArea = 0;
        for (PhotonTrackedTarget sampleTarget : result.getTargets()){ 
          //loops through every sample target in results.getTargets()
          //if the sample target's area is bigger than the biggestTargetArea, then the sample target
          // is set to the target, and the biggest Target Area is set to the sample's target area
          if (sampleTarget.getArea() > biggestTargetArea){
            biggestTargetArea = sampleTarget.getArea();
            target = sampleTarget;
          }
        }

        targetYaw = aprilTagFieldLayout.getTagPose(target.getFiducialId()).get().getRotation().getZ();

        targetPose = aprilTagFieldLayout.getTagPose(target.getFiducialId()).get().toPose2d();

        targetFound = true;
      }
      else {
        targetFound = false;
      }
    }

    SmartDashboard.putBoolean("Target Found", targetFound);
    SmartDashboard.putNumber("Target Distance Meters", targetDistanceMeters);
    SmartDashboard.putNumber("Target Yaw (radians)", targetYaw);
    SmartDashboard.putNumber("Target X Distance", targetTranslation2d.getX());
    SmartDashboard.putNumber("Target Y Distance", targetTranslation2d.getY());

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  // returns bool if camera within tolerance to AprilTag
  public boolean isWithinTarget(Pose2d currentPose){
    SmartDashboard.putBoolean("inTarget", isWithinTarget(currentPose,1, 1));
    return isWithinTarget(currentPose,1, 1);
  }

  public Trigger isWithinTargetTrigger(Supplier<Pose2d> currentPoseSupplier) {
    return new Trigger(() -> isWithinTarget(currentPoseSupplier.get()));
  }

  public boolean isWithinTarget(Pose2d currentPose, double toleranceX, double toleranceY) {
    if (targetPose != null) {
      targetTranslation2d = new Translation2d(currentPose.getX() - targetPose.getX(), currentPose.getY() - targetPose.getY());
    
    return (targetFound
            && Math.abs(targetTranslation2d.getX()) < toleranceX
            && Math.abs(targetTranslation2d.getY()) < toleranceY);
    } else {
      return false;
    }
    
  }
}
