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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.hardware.Pigeon2;

import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;



public class VisionSubsystem extends SubsystemBase {
  private final PhotonCamera m_camera;
  private final Pigeon2 m_gyro;
  // private final LEDSubsystem m_ledSubsystem;
  private Translation2d targetTranslation2d = new Translation2d(0,0); // distance to the target; updated every periodic() call if target is found 
  private boolean targetFound; // true if the translation2d was updated last periodic() call
  private Pose2d targetPose;
  private Pose2d currentPose; // current robot pose, updates periodically

  private AprilTagFieldLayout aprilTagFieldLayout;

  private double distToTargetX;
  private double distToTargetY;

  /** Creates a new Vision Subsystem. */
  public VisionSubsystem(Pigeon2 pigeon) {
    m_camera = new PhotonCamera(VisionConstants.CAMERA_NAME);
    m_gyro = pigeon;

    aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    
    distToTargetX = 1;
    distToTargetY = 1;
  }

  public Translation2d getRobotTranslationToTag() {
    return targetFound ? targetTranslation2d.plus(VisionConstants.ROBOT_TO_CAM) : new Translation2d(0,0); // only return translation2d if target was found
  }

  @Override
  public void periodic() {

    targetFound = false; 
    double targetDistanceMeters = 0.0;

    var results = m_camera.getAllUnreadResults();

    if (!results.isEmpty()) {
      // Camera processed a new frame since last
      // Get the last one in the list.
      var result = results.get(results.size() - 1);

      if (result.hasTargets()) {
        PhotonTrackedTarget target = result.getBestTarget();
        targetPose = aprilTagFieldLayout.getTagPose(target.getFiducialId()).get().toPose2d();

        // calculate currentPose of robot relative to field
        currentPose = PhotonUtils.estimateFieldToRobot(VisionConstants.CAMERA_HEIGHT, VisionConstants.CAMERA_PITCH, VisionConstants.TARGET_HEIGHT, VisionConstants.TARGET_PITCH, new Rotation2d(target.getYaw()), new Rotation2d(m_gyro.getYaw().getValue()), targetPose, new Transform2d(VisionConstants.ROBOT_TO_CAM, new Rotation2d(0)));

        // euclidean distance between currentPose and targetPose
        targetDistanceMeters = PhotonUtils.getDistanceToPose(currentPose, targetPose);

        // translation 2d between currentPose and targetPose
        targetTranslation2d = new Translation2d(currentPose.getX() - targetPose.getX(), currentPose.getY() - targetPose.getY());

        targetFound = true;
      }
    }

    SmartDashboard.putBoolean("Target Found", targetFound);
    SmartDashboard.putNumber("Target Distance Meters", targetDistanceMeters);
    SmartDashboard.putBoolean("inTarget", isWithinTarget());
    SmartDashboard.putNumber("Target X Distance", targetTranslation2d.getX());
    SmartDashboard.putNumber("Target Y Distance", targetTranslation2d.getY());

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  // returns bool if camera within tolerance to AprilTag
  public boolean isWithinTarget(){
    return isWithinTarget(1, 1);
  }

  public Trigger isWithinTargetTrigger() {
    return new Trigger(() -> isWithinTarget());
  }

  public boolean isWithinTarget(double toleranceX, double toleranceY) {
    return (targetFound
            && Math.abs(targetTranslation2d.getX() - distToTargetX) < toleranceX
            && Math.abs(targetTranslation2d.getY() - distToTargetY) < toleranceY);
  }
}
