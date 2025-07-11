package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants.VisionConstants;

public class VisionUtils {

    private static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    private static List<Pose2d> reefAprilTagPoses = new ArrayList<>();

    static {
        // create the list of apriltag poses
        List<Integer> reefAprilTagIDs = new ArrayList<>(Arrays.asList(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22));
        for (Integer id : reefAprilTagIDs) {
        reefAprilTagPoses.add(aprilTagFieldLayout.getTagPose(id).get().toPose2d());
        }
    }
    
    public static Pose2d getClosestReefAprilTag(Pose2d robotPose) {
        return robotPose.nearest(reefAprilTagPoses);
    } 


    public static boolean poseIsSane(Pose3d pose) {
        return pose.getZ() < VisionConstants.MAX_VISION_POSE_Z 
        && pose.getRotation().getX() < VisionConstants.MAX_VISION_POSE_ROLL 
        && pose.getRotation().getY() < VisionConstants.MAX_VISION_POSE_PITCH;

    }

    public static boolean zIsSane(Pose3d pose) {
        return pose.getZ() < VisionConstants.MAX_VISION_POSE_Z;

    }

    public static boolean rollIsSane(Pose3d pose) {
        return pose.getRotation().getX() < VisionConstants.MAX_VISION_POSE_ROLL;

    }

    public static boolean pitchIsSane(Pose3d pose) {
        return pose.getRotation().getY() < VisionConstants.MAX_VISION_POSE_PITCH;

    }

}
