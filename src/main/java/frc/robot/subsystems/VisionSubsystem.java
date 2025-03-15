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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
  private Matrix<N3,N1> curStdDevs = VisionConstants.kSingleTagStdDevs;

  private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  private List<Pose2d> reefAprilTagPoses = new ArrayList<>();
  private PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.ROBOT_TO_CAM_3D);


  private double distToTargetX;
  private double distToTargetY;

  /** Creates a new Vision Subsystem. */
  public VisionSubsystem(Pigeon2 pigeon) {
    m_gyro = pigeon;
    
    distToTargetX = 1;
    distToTargetY = 1;    

    // create the list of apriltag poses
    List<Integer> reefAprilTagIDs = new ArrayList<>(Arrays.asList(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22));
    for (Integer id : reefAprilTagIDs) {
      reefAprilTagPoses.add(aprilTagFieldLayout.getTagPose(id).get().toPose2d());
    }

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

        if (target.getPoseAmbiguity() > VisionConstants.MAX_POSE_AMBIGUITY) {
          SmartDashboard.putString("vision/targetState", "targetDiscardedAmbiguity");
          targetFound = false;
        } else {
          SmartDashboard.putString("vision/targetState", "targetFound");
          targetFound = true;
        }
        
        targetYaw = aprilTagFieldLayout.getTagPose(target.getFiducialId()).get().getRotation().getZ();
        targetPose = aprilTagFieldLayout.getTagPose(target.getFiducialId()).get().toPose2d();
      } else {
        SmartDashboard.putString("vision/targetState", "noTarget");
        targetFound = false;
      }

      if (targetFound) {
        // update the estimated robot pose if the pose estimator outputs something
        Optional<EstimatedRobotPose> poseEstimatorOutput = poseEstimator.update(result);

        // update standard deviation based on dist 
        this.updateEstimationStdDevs(poseEstimatorOutput, result.getTargets());

        if (poseEstimatorOutput.isPresent() && poseIsSane(poseEstimatorOutput.get().estimatedPose)) {
          estimatedRobotPose = poseEstimatorOutput.get(); 
        }
      }
    }

    SmartDashboard.putNumberArray("Targets Seen", targetIds.stream().mapToDouble(Integer::doubleValue).toArray());
    SmartDashboard.putNumberArray("Target Pose Ambiguities", targetPoseAmbiguity.stream().mapToDouble(Double::doubleValue).toArray());
    SmartDashboard.putBoolean("Target Found", targetFound);
    SmartDashboard.putNumber("Target Distance Meters", targetDistanceMeters);
    SmartDashboard.putNumber("Target Yaw (radians)", targetYaw);
    SmartDashboard.putNumber("Target X Distance", targetTranslation2d.getX());
    SmartDashboard.putNumber("Target Y Distance", targetTranslation2d.getY());

  }

  private boolean poseIsSane(Pose3d pose) {
    return pose.getZ() < VisionConstants.MAX_VISION_POSE_Z 
    && pose.getRotation().getX() < VisionConstants.MAX_VISION_POSE_ROLL 
    && pose.getRotation().getY() < VisionConstants.MAX_VISION_POSE_PITCH;

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

  private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
        // No pose input. Default to single-tag std devs
        curStdDevs = VisionConstants.kSingleTagStdDevs;
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
      } 
      else if (numTags == 1 && avgDist > 4){  
        curStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
      }
      else {
        var unscaledStdDevs = numTags > 1 ? VisionConstants.kMultiTagStdDevs:VisionConstants.kSingleTagStdDevs;
      
        avgDist /= numTags;
        // increase std devs based on (average) distance
        curStdDevs = unscaledStdDevs.times(1 + (avgDist * avgDist / 30));
      }
    }
  }

  public Pose2d getClosestReefAprilTag(Pose2d robotPose) {
    return robotPose.nearest(reefAprilTagPoses);
  } 
}
