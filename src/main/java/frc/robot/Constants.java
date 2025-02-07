// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Translation2d;


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

  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCopilotControllerPort = 1;
    public static final int kTestControllerPort = 2;

    public static final double JOYSTICK_DEADBAND = 0.1;
    public static final double SLOWMODE_FACTOR = 0.1;
  }

  public static class DrivetrainConstants {
    // Not tuned
    public static final double ROBOT_ROTATION_P = 5; // 11.507 from rotation sys-id @PF 1/13
    public static final double ROBOT_ROTATION_I = 0;
    public static final double ROBOT_ROTATION_D = 0; // 0.10877 from rotation sys-id @PF 1/13
    public static final double ROBOT_POSITION_P = 5;
    public static final double ROBOT_POSITION_I = 0;
    public static final double ROBOT_POSITION_D = 0;

    public static final double WHEEL_RADIUS_FUDGE_FACTOR = 0.96153846153; // approximated @ PF session 1/13
  }

  public static class ElevatorConstants {
    public static final int ELEVATOR_MOTOR_ID = 21; // placeholder
    public static final int MOTOR_STATOR_LIMIT = 80; // needs to be tuned
  
    public static final int COUNTS_PER_REV = 42; // may need to be updated

    public static final double METERS_PER_ROTATION = 0.0; 
    
    public static final double MAX_SPEED = 0.0; 
    public static final double SPEED_LOWER_LIMIT = 0.0;
    public static final double SPEED_UPPER_LIMIT = 0.0;

    // not tuned
    public static final double kS = 0;
    public static final double kG = 0;
    public static final double kV = 12/MAX_SPEED; // may need to be updated
    public static final double kA = 0;

    public static final double kP = 0; 
    public static final double kI = 0;
    public static final double kD = 0; 

    public static final double kMaxV = 0;
    public static final double kMaxA = 0; 

  }

  public static class ArmConstants {
    public static final int ARM_MOTOR_ID = 22; // placeholder
    public static final int MOTOR_STATOR_LIMIT = 80; // needs to be tuned

    public static final int COUNTS_PER_REV = 42; // may need to be updated

    public static final double DEGREES_PER_ROTATION = 0.0; 
    public static final double MAX_SPEED = 0.0; 
    public static final double SPEED_LOWER_LIMIT = 0.0;
    public static final double SPEED_UPPER_LIMIT = 0.0;

    public static final double kS = 0;
    public static final double kG = 0;
    public static final double kV = 12/MAX_SPEED; // may need to be updated
    public static final double kA = 0;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    
    public static final double kMaxV = 0;
    public static final double kMaxA = 0; 
  }

  public static class CollarConstants {
    public static final int COLLAR_MOTOR_ID = 23; // placeholder
    public static final int MOTOR_STATOR_LIMIT = 20; // needs to be tuned

    public static final int COUNTS_PER_REV = 42; // may need to be updated

    public static final double METERS_PER_ROTATION = 0.0; 
    public static final double MAX_SPEED = 0.0; 
    public static final double SPEED_LOWER_LIMIT = 0.0;
    public static final double SPEED_UPPER_LIMIT = 0.0;
  }

  public static class RampConstants {

  }
  
  public static class LEDConstants {
    public static final int KPORT = 0;
    public static final int KLENGTH = 30;
  }

  public static class VisionConstants {
    public static final String CAMERA_NAME = "max_camera";  

    public static final Translation2d ROBOT_TO_CAM = 
      new Translation2d(0.3048, 0); // in meters from center of robot to 2x4 camera mount
    
    public static final double CAMERA_HEIGHT = 0.0951738; // in meters from floor to camera center
    public static final double CAMERA_PITCH = 0; // in radians, bogus
    public static final double TARGET_HEIGHT = 0.3048; // in meters to the middle of the apriltag on reef
  }

  public static class IntakeConstants {
    public static final int LASER_CAN_ID = 14;
  }

  public static class RobotStateConstants {
    public static final double C1_ELEVATOR_POS = 0.0;
    public static final double C1_ARM_POS = 0.0;
    public static final double C2_ELEVATOR_POS = 0.0;
    public static final double C2_ARM_POS = 0.0;
    public static final double C3_ELEVATOR_POS = 0.0;
    public static final double C3_ARM_POS = 0.0;
    public static final double C4_ELEVATOR_POS = 0.0;
    public static final double C4_ARM_POS = 0.0;
    public static final double A2_ELEVATOR_POS = 0.0;
    public static final double A2_ARM_POS = 0.0;
    public static final double A3_ELEVATOR_POS = 0.0;
    public static final double A3_ARM_POS = 0.0;
    public static final double IN_ELEVATOR_POS = 0.0;
    public static final double IN_ARM_POS = 0.0;
  }
}
