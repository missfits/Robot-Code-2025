// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;

import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.VisionUtils;



public class VisionSubsystem extends SubsystemBase {
  private final PhotonCamera m_camera;
  private final String m_cameraName;
  // private final LEDSubsystem m_ledSubsystem;
  private Translation2d targetTranslation2d = new Translation2d(0,0); // distance to the target; updated every periodic() call if target is found 
  private boolean targetFound; // true if the translation2d was updated last periodic() call
  private Pose2d targetPose;
  private double targetYaw; // in radians, relative to field
  private EstimatedRobotPose estimatedRobotPose; 
  private Matrix<N3,N1> curStdDevs = VisionConstants.kSingleTagStdDevs;

  private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  private PhotonPoseEstimator poseEstimator;

  private boolean isNewResult = false;

  private ArrayList<Pose2d> lastEstPoses = new ArrayList<>();

  /** Creates a new Vision Subsystem. */
  public VisionSubsystem(String cameraName, Transform3d robotToCam) {
    m_cameraName = cameraName;
    m_camera = new PhotonCamera(cameraName);
    poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
  }

  
  public String getCameraName() {
    return m_cameraName;
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

  public Matrix<N3, N1> getCurrentStdDevs(){
    return curStdDevs;
  }

  @Override
  public void periodic() {

    double targetDistanceMeters = 0.0;

    var results = m_camera.getAllUnreadResults();

    ArrayList<Integer> targetIds = new ArrayList<Integer>();
    ArrayList<Double> targetPoseAmbiguity = new ArrayList<Double>();

    if (!results.isEmpty()) {
      // Camera processed a new frame since last
      // Get the last one in the list.
      var result = results.get(results.size() - 1);
      isNewResult = true;

      if (result.hasTargets()) {
        PhotonTrackedTarget target = null;
        double biggestTargetArea = 0;

        for (PhotonTrackedTarget sampleTarget : result.getTargets()){
          targetIds.add(sampleTarget.getFiducialId());
          targetPoseAmbiguity.add(sampleTarget.getPoseAmbiguity());
          //loops through every sample target in results.getTargets()
          //if the sample target's area is bigger than the biggestTargetArea, then the sample target
          // is set to the target, and the biggest Target Area is set to the sample's target area
          if (sampleTarget.getArea() > biggestTargetArea){
            biggestTargetArea = sampleTarget.getArea();
            target = sampleTarget;
          }
        }

        // gets target with lowest reprojection error
        SmartDashboard.putNumber("vision/" + m_camera + "Alternate Target X", target.getBestCameraToTarget().getX());
        SmartDashboard.putNumber("vision/" + m_camera + "Alternate Target Y", target.getBestCameraToTarget().getY());

        if (target.getPoseAmbiguity() > VisionConstants.MAX_POSE_AMBIGUITY) {
          SmartDashboard.putString("vision/" + m_cameraName + "/targetState", "targetDiscardedAmbiguity");
          targetFound = false;
        } else {
          SmartDashboard.putString("vision/" + m_cameraName + "/targetState", "targetFound");
          targetFound = true;
        }
        
        targetYaw = aprilTagFieldLayout.getTagPose(target.getFiducialId()).get().getRotation().getZ();
        targetPose = aprilTagFieldLayout.getTagPose(target.getFiducialId()).get().toPose2d();
      } else {
        SmartDashboard.putString("vision/" + m_cameraName + "/targetState", "noTarget");
        targetFound = false;
      }

      if (targetFound) {
        // update the estimated robot pose if the pose estimator outputs something
        Optional<EstimatedRobotPose> poseEstimatorOutput = poseEstimator.update(result);

        // update standard deviation based on dist 
        this.updateEstimationStdDevs(poseEstimatorOutput, result.getTargets());

        if (poseEstimatorOutput.isPresent()) {
          if (VisionUtils.poseIsSane(poseEstimatorOutput.get().estimatedPose)){
            SmartDashboard.putString("vision/" + m_cameraName + "/targetState", "targetFound");
            estimatedRobotPose = poseEstimatorOutput.get(); 
  
            // update our last n poses
            lastEstPoses.add(estimatedRobotPose.estimatedPose.toPose2d());
            if (lastEstPoses.size() > VisionConstants.NUM_LAST_EST_POSES) {
              lastEstPoses.remove(0);
            }
          }
          else {
            SmartDashboard.putString("vision/" + m_cameraName + "/targetState", "poseNOTsane");
            targetFound = false;

            
          }

          SmartDashboard.putBoolean("vision/" + m_cameraName + "/zIsSane", VisionUtils.zIsSane(poseEstimatorOutput.get().estimatedPose));
          SmartDashboard.putBoolean("vision/" + m_cameraName + "/rollIsSane", VisionUtils.rollIsSane(poseEstimatorOutput.get().estimatedPose));
          SmartDashboard.putBoolean("vision/" + m_cameraName + "/pitchIsSane", VisionUtils.pitchIsSane(poseEstimatorOutput.get().estimatedPose));


        }
      }
      else{
        isNewResult = false;
      }

    }

    SmartDashboard.putNumberArray("vision/" + m_cameraName + "/Targets Seen", targetIds.stream().mapToDouble(Integer::doubleValue).toArray());
    SmartDashboard.putNumberArray("vision/" + m_cameraName + "/Target Pose Ambiguities", targetPoseAmbiguity.stream().mapToDouble(Double::doubleValue).toArray());
    SmartDashboard.putBoolean("vision/" + m_cameraName + "/Target Found", targetFound);
    SmartDashboard.putNumber("vision/" + m_cameraName + "/Target Distance Meters", targetDistanceMeters);
    SmartDashboard.putNumber("vision/" + m_cameraName + "/Target Yaw (radians)", targetYaw);
    SmartDashboard.putNumber("vision/" + m_cameraName + "/Target X Distance", targetTranslation2d.getX());
    SmartDashboard.putNumber("vision/" + m_cameraName + "/Target Y Distance", targetTranslation2d.getY());

    SmartDashboard.putBoolean("vision/" + m_cameraName + "/isEstPoseJumpy", isEstPoseJumpy());
    SmartDashboard.putNumberArray("vision/" + m_cameraName + "/standardDeviations", curStdDevs.getData());

    SmartDashboard.putBoolean("vision/" + m_cameraName + "/isConnected", m_camera.isConnected());
    SmartDashboard.putBoolean("vision/" + m_cameraName + "/isNewResult", getIsNewResult());

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


  public boolean getIsNewResult(){
    return isNewResult;
  }

  // returns bool if camera within tolerance to AprilTag
  public boolean isWithinTarget(Pose2d currentPose){
    SmartDashboard.putBoolean("vision/" +  m_cameraName + "/inTarget", isWithinTarget(currentPose,1, 1));
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

  private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
        // No pose input. Default to single-tag std devs
        curStdDevs = VisionConstants.kSingleTagStdDevs;
        SmartDashboard.putNumber("vision/" + m_cameraName + "/standardDeviation-average-distance", Double.MAX_VALUE);
        SmartDashboard.putString("vision/" + m_cameraName + "/standardDeviation-state", "empty");
    } 
    else {
      // Pose present. Start running Heuristic
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an average-distance metric
      for (var tgt : targets) {
          var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
          if (tagPose.isEmpty()) continue;
          numTags++;
          avgDist +=
                  tagPose
                          .get()
                          .toPose2d()
                          .getTranslation()
                          .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0) {
          // No tags visible. Default to single-tag std devs
          curStdDevs = VisionConstants.kSingleTagStdDevs;
          SmartDashboard.putString("vision/" + m_cameraName + "/standardDeviation-state", "no tags visible");
      } 
      else if (numTags == 1 && avgDist > VisionConstants.VISION_DISTANCE_DISCARD){  
        curStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        SmartDashboard.putString("vision/" + m_cameraName + "/standardDeviation-state", "target too far");
      }
      else {
        var unscaledStdDevs = numTags > 1 ? VisionConstants.kMultiTagStdDevs:VisionConstants.kSingleTagStdDevs;
      
        avgDist /= numTags;
        // increase std devs based on (average) distance
        curStdDevs = unscaledStdDevs.times(1 + (avgDist * avgDist / 30));
        SmartDashboard.putString("vision/" + m_cameraName + "/standardDeviation-state", "good :)");
      }
      SmartDashboard.putNumber("vision/" + m_cameraName + "/standardDeviation-average-distance", avgDist);
    }
  }

  public boolean isEstPoseJumpy() {
    if (lastEstPoses.size() < VisionConstants.NUM_LAST_EST_POSES) {
      return true;
    }

    double totalDistance = 0;

    for (int i = 0; i<lastEstPoses.size()-1; i++) {
      // add distance between ith pose and i+1th pose
      totalDistance += Math.abs(lastEstPoses.get(i).minus(lastEstPoses.get(i+1)).getTranslation().getNorm());
    }
    
    double avgDist = totalDistance/lastEstPoses.size();
    SmartDashboard.putNumber("vision/" + m_cameraName + "/avgDistBetweenLastEstPoses", avgDist);

    return avgDist > VisionConstants.MAX_AVG_DIST_BETWEEN_LAST_EST_POSES;
  }

}
