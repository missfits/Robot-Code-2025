// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.RobotContainer.RobotName;
import edu.wpi.first.math.util.Units;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class RobotConstants {
    public static final int DISABLED_COAST_DELAY = 10; // in secs

    public static final boolean COMPETITION_MODE = false;
  }

  public static class FieldConstants {
    public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);


  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCopilotControllerPort = 1;
    public static final int kTestControllerPort = 2;

    public static final double TRANSLATION_JOYSTICK_DEADBAND = 0.1;
    public static final double ROTATION_JOYSTICK_DEADBAND = 0.1;
    public static final double SLOWMODE_FACTOR = 0.3;

    public static final double SLEW_RATE_LIMIT = 1.5; // limits how fast operator input (-1 to 1) 
  }

  public static class DrivetrainConstants {

    public static final double ROBOT_SIZE_X =  RobotContainer.name == RobotName.DYNAMENE ? Units.inchesToMeters(35) : 0.66675; // in meters, including bumpers (est. 26.25in for ceridwen)
    // Not tuned
    public static final double ROBOT_ROTATION_P = 5; // 11.507 from rotation sys-id @PF 1/13
    public static final double ROBOT_ROTATION_I = 0;
    public static final double ROBOT_ROTATION_D = 0; // 0.10877 from rotation sys-id @PF 1/13
    
    public static final double ROBOT_POSITION_P = 10;
    public static final double ROBOT_POSITION_I = 0;
    public static final double ROBOT_POSITION_D = 0;

    public static final double AUTOALIGN_POSITION_P = 4;
    public static final double AUTOALIGN_POSITION_I = 0;
    public static final double AUTOALIGN_POSITION_D = 0;

    public static final double TO_TARGET_POSITION_P = 2.5;
    public static final double TO_TARGET_POSITION_I = 0;
    public static final double TO_TARGET_POSITION_D = 0;
  
    public static final double CERIDWEN_WHEEL_RADIUS_FUDGE_FACTOR = 0.96153846153; // approximated @ PF session 1/13
    public static final double DYNAMENE_WHEEL_RADIUS_FUDGE_FACTOR = 0.97205; // approximated @ PF session 1/13

  }

  public static class ElevatorConstants {
    public static final int ELEVATOR_MOTOR_ID = 21; // placeholder
  
    public static final int COUNTS_PER_REV = 42; // may need to be updated

    public static final double INITIAL_POSITION = 0;
    public static final double METERS_PER_ROTATION = Units.inchesToMeters(1.751 * Math.PI/5); // temp
    public static final double MAX_SPEED = METERS_PER_ROTATION*100; // motor rotates at ~ 100 rot/sec at free speed
    public static final double SPEED_LOWER_LIMIT = 0.0;
    public static final double SPEED_UPPER_LIMIT = 0.0;
    
    public static final double POSITION_LOWER_LIMIT = 0.005;
    public static final double POSITION_UPPER_LIMIT = 0.71;
    
    public static final int MOTOR_STATOR_LIMIT = 60; // needs to be tuned

    // not tuned
    public static double kS = 0;
    public static double kG = 0.17;
    public static double kV = 3; 
    public static double kA = 0.6;

    public static double kP = 125; 
    public static double kI = 15;
    public static double kD = 0; 

    public static double kMaxV = 3;
    public static double kMaxA = 6; 

    public static final double MANUAL_MOVE_MOTOR_SPEED = 1.5;

    public static final double MAX_POSITION_TOLERANCE = 0.005;
    public static final double PROFILE_TOLERANCE = 0.08;

    public static final double MIN_POS_ARM_CLEAR = 0.28; 

    public static final double MIN_HEIGHT_TO_BE_TALL = 0.4; // for slew rate limiting when elevator is tall

    public static final double MIN_HEIGHT_TO_BE_BELOW_L4 = 0.68;
    
  }

  public static class ArmConstants {
    public static final int ARM_MOTOR_ID = 22; // placeholder

    public static final int COUNTS_PER_REV = 42; // may need to be updated

    public static final double INITIAL_POSITION = 0; // facing down
    public static final double POSITION_OFFSET = Math.PI/2; // difference between pid 0 (horizontal) and our 0 (down)
    public static final double DEGREES_PER_ROTATION = 1.0/27.0*15.0/54.0*360.0; 
    public static final double MAX_SPEED = Math.toRadians(DEGREES_PER_ROTATION*100); 
    public static final double SPEED_LOWER_LIMIT = 0.0;
    public static final double SPEED_UPPER_LIMIT = 0.0;
    public static final int MOTOR_STATOR_LIMIT = 60; // needs to be tuned

    public static double kS = 0.15;
    public static double kG = 0;
    public static double kV = 0.6;
    public static double kA = 0.0125;

    public static double kP = 75;
    public static double kI = 0;
    public static double kD = 0;
    
    public static double kMaxV = 9;
    public static double kMaxA = 17; 


    public static final double MANUAL_MOVE_MOTOR_SPEED = 3.0;

    public static final double MAX_POSITION_TOLERANCE = 0.025; // equal to 1 degree
    public static final double PROFILE_TOLERANCE = Math.toRadians(20);

    public static final double MIN_POS_ELEVATOR_CLEAR = -0.3; // accounts for when coral inside collar
    public static final double LOWER_INSIDE_ROBOT_BOUND = 0;
    public static final double UPPER_INSIDE_ROBOT_BOUND = -Math.PI;

  }

  public static class CollarConstants {
    public static final int COLLAR_MOTOR_ID = 23; // placeholder
    public static final int MOTOR_STATOR_LIMIT = 20; // needs to be tuned

    public static final int COUNTS_PER_REV = 42; // may need to be updated

    public static final double METERS_PER_ROTATION = 1; 
    public static final double MAX_SPEED = 0.0; 
    public static final double SPEED_LOWER_LIMIT = 0.0;
    public static final double SPEED_UPPER_LIMIT = 0.0;

    public static final double OUTTAKE_MOTOR_SPEED = 8.0;
    public static final double SLOW_OUTTAKE_MOTOR_SPEED = 2.0;

    
    public static final double INTAKE_MOTOR_SPEED = 6.0;
    public static final double INTAKE_SECONDARY_MOTOR_SPEED = 3.0;
    public static final double INTAKE_SECONDARY_BACK_MOTOR_SPEED = -6;
    public static final double INTAKE_BACKWARDS_TIMEOUT = 0.2;

    public static final double BACKWARDS_MOTOR_SPEED = -1.0;

    public static final double INTAKE_STOP_OFFSET = 0; // in seconds
  }

  public static class ClimberConstants {

    public static final int CLIMBER_MOTOR_ID = 16;

    public static final double MOTOR_STATOR_LIMIT = 40; // needs to be updated lol

    public static final double MANUAL_MOVE_MOTOR_SPEED = 8.0;
    public static final double DEPLOY_MOVE_MOTOR_SPEED = 12.0; 
    public static final double LIFT_MOVE_MOTOR_SPEED = 9.0; 
    public static final double DEGREES_PER_ROTATION = 360.0/690.0;

    public static final double INITIAL_POSITION = 0.0; // down
    public static final double DEPLOY_POSITION = -1.98; 
    public static final double LIFT_POSITION = -4.50; 
  }
  
  public static class LEDConstants {
    public static final int KPORT = 0;
    public static final int KLENGTH = 60;

    public static final double BLINK_TIME = 1; // in seconds for after intake/outtake
  }

  public static class AutoAlignConstants {
    public static final double REEF_OFFSET_RIGHT = Units.inchesToMeters(3.5);
    public static final double REEF_OFFSET_LEFT = Units.inchesToMeters(9.5);

    public static final double kMaxV = 2; // to be tuned
    public static final double kMaxA = 2.5; // to be tuned

    public static final double kMaxIntermediateV = 2; // to be tuned
    public static final double kMaxIntermediateA = 1.5; // to be tuned

    public static final double INTERMEDIATE_POS_DIST = Units.inchesToMeters(5);
  }

  public static class VisionConstants {
    public static final String CAMERA1_NAME = "beam_camera";  
    public static final String CAMERA2_NAME = "swerve_camera";  

    public static final double ROBOT_TO_CAM1_X = RobotContainer.name == RobotName.DYNAMENE ? Units.inchesToMeters(-2) : 0.31115 ; // in meters from center of robot 
    public static final double ROBOT_TO_CAM1_Y = RobotContainer.name == RobotName.DYNAMENE ? Units.inchesToMeters(-1) : -0.0508; // in meters from center of robot 
    public static final double ROBOT_TO_CAM1_Z = RobotContainer.name == RobotName.DYNAMENE ? Units.inchesToMeters(17) : 0.1397; // in meters from the floor?
    

    public static final double ROBOT_TO_CAM2_X = RobotContainer.name == RobotName.DYNAMENE ? Units.inchesToMeters(13-4.75) : 0 ; // in meters from center of robot 
    public static final double ROBOT_TO_CAM2_Y = RobotContainer.name == RobotName.DYNAMENE ? Units.inchesToMeters(13-3.125) : 0; // in meters from center of robot 
    public static final double ROBOT_TO_CAM2_Z = RobotContainer.name == RobotName.DYNAMENE ? Units.inchesToMeters(7.5) : 0; // in meters from the floor?
    

    // default vision standard deviation
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(6, 6, 4);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 0.3);

    public static final double MAX_POSE_AMBIGUITY = 0.2;
    public static final double MAX_VISION_POSE_DISTANCE = 1;
    public static final double MAX_VISION_POSE_Z = 0.1;
    public static final double MAX_VISION_POSE_ROLL = 0.05; // in radians
    public static final double MAX_VISION_POSE_PITCH = 0.05; // in radians
    public static final double VISION_DISTANCE_DISCARD = 10; 

    public static final double VISION_ALIGNMENT_DISCARD = Units.inchesToMeters(1); // in meters


    public static final Translation2d ROBOT_TO_CAM1 = 
      new Translation2d(ROBOT_TO_CAM1_X, ROBOT_TO_CAM1_Y); // in meters from center of robot to 2x4 camera mount

    public static final Translation2d ROBOT_TO_CAM2 = 
      new Translation2d(ROBOT_TO_CAM2_X, ROBOT_TO_CAM2_Y); // in meters from center of robot to 2x4 camera mount

    public static final Transform3d ROBOT_TO_CAM1_3D = 
      new Transform3d(new Translation3d(ROBOT_TO_CAM1_X, ROBOT_TO_CAM1_Y, ROBOT_TO_CAM1_Z), new Rotation3d(0,0,0)); // in meters from center of robot to 2x4 camera mount
    
    public static final Transform3d ROBOT_TO_CAM2_3D = 
      new Transform3d(new Translation3d(ROBOT_TO_CAM2_X, ROBOT_TO_CAM2_Y, ROBOT_TO_CAM2_Z), new Rotation3d(0,0,-Math.PI/9)); // in meters from center of robot to 2x4 camera mount
  

    public static final double CAMERA_HEIGHT = 0.0951738; // in meters from floor to camera center
    public static final double CAMERA_PITCH = 0; // in radians, bogus
    public static final double TARGET_HEIGHT = 0.3048; // in meters to the middle of the apriltag on reef
    public static final double TARGET_PITCH = 0;

    public static final double MAX_AVG_DIST_BETWEEN_LAST_EST_POSES = 0.3; // in meters 
    public static final int NUM_LAST_EST_POSES = 3;
  }

  public static class LaserCanConstants {
    public static final int LASER_CAN_RAMP_OUT_ID = 14;
    public static final int LASER_CAN_RAMP_IN_ID = 15;

    public static final double MIN_CORAL_SEEN_DISTANCE_RAMP_OUT = 20; // in mm
    public static final double MIN_CORAL_SEEN_DISTANCE_RAMP_IN = 300; // in mm

  }

  public static class RobotStateConstants {
    public static final double C1_ELEVATOR_POS = ElevatorConstants.INITIAL_POSITION; 
    public static final double C1_ARM_POS = ArmConstants.INITIAL_POSITION;

    public static final double C2_ELEVATOR_POS = 0.3611; // from 3/1
    public static final double C2_ARM_POS = 0.3370; // from 3/1

    public static final double C2_ELEVATOR_POS_WITH_CORAL = 0.3172; // from 3/1
    public static final double C2_ARM_POS_WITH_CORAL = 1.0371; // from 3/1

    public static final double C3_ELEVATOR_POS = 0.670; // from 3/1
    public static final double C3_ARM_POS = 0.5078; // from 3/1

    public static final double C3_ELEVATOR_POS_WITH_CORAL = 0.7124; // from 3/1
    public static final double C3_ARM_POS_WITH_CORAL = 1.0515; // from 3/1

    public static final double C4_ELEVATOR_POS = 0.7043; // from 3/1
    public static final double C4_ARM_POS = -3.211; // from 3/1
    
    public static final double C4_ELEVATOR_POS_WITH_CORAL = 0.7026; // from 3/1
    public static final double C4_ARM_POS_WITH_CORAL = -3.2239; // from 3/1

  
    public static final double A2_ELEVATOR_POS = 0.01; // from 3/25
    public static final double A2_ARM_POS = 0.8532; // from 3/1

    public static final double A3_ELEVATOR_POS = 0.3611; // from 3/1
    public static final double A3_ARM_POS = 0.9547; // from 3/1
    
    public static final double IN_ELEVATOR_POS = 0.024;
    public static final double IN_ARM_POS = -0.43;

    public static final double CLIMB_ELEVATOR_POS = 0.01;
    public static final double CLIMB_ARM_POS = Math.PI/4; // 45 degrees
  }
}
