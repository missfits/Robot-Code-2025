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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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
  private ArrayList<LocalizationCamera> cameras = new ArrayList<>();
  private ArrayList<PhotonTrackedTarget> targetResults = new ArrayList<>();

  /** Creates a new Vision Subsystem. */
  public VisionSubsystem() {
    cameras.add(new LocalizationCamera(VisionConstants.CAMERA1_NAME, VisionConstants.ROBOT_TO_CAM1_3D));
    cameras.add(new LocalizationCamera(VisionConstants.CAMERA2_NAME, VisionConstants.ROBOT_TO_CAM2_3D));
  }

  public ArrayList<LocalizationCamera> getLocalizationCameras(){
    return cameras;
  }

  @Override
  public void periodic() {
    for (LocalizationCamera cam : cameras){
      cam.findTarget();
      
      if (cam.getTargetFound()){
        targetResults.add(cam.getTarget());
      }
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
