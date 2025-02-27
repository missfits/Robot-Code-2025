// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
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

  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCopilotControllerPort = 1;
    public static final int kTestControllerPort = 2;

    public static final double JOYSTICK_DEADBAND = 0.1;
    public static final double SLOWMODE_FACTOR = 0.1;
  }

  public static class DrivetrainConstants {

    public static final double ROBOT_SIZE_X =  RobotContainer.name == RobotName.DYNAMENE ? 0.8 : 0.66675; // in meters, including bumpers (est. 26.25in for ceridwen)
    // Not tuned
    public static final double ROBOT_ROTATION_P = 5; // 11.507 from rotation sys-id @PF 1/13
    public static final double ROBOT_ROTATION_I = 0;
    public static final double ROBOT_ROTATION_D = 0; // 0.10877 from rotation sys-id @PF 1/13
    public static final double ROBOT_POSITION_P = 5;
    public static final double ROBOT_POSITION_I = 0;
    public static final double ROBOT_POSITION_D = 0;

    public static final double CERIDWEN_WHEEL_RADIUS_FUDGE_FACTOR = 0.96153846153; // approximated @ PF session 1/13
    public static final double DYNAMENE_WHEEL_RADIUS_FUDGE_FACTOR = 0.96153846153; // approximated @ PF session 1/13

  }

  public static class ElevatorConstants {
    public static final int ELEVATOR_MOTOR_ID = 21; // placeholder
  
    public static final int COUNTS_PER_REV = 42; // may need to be updated

    public static final double INITIAL_POSITION = 0;
    public static final double METERS_PER_ROTATION = Units.inchesToMeters(1.751 * Math.PI/5); // temp
    public static final double MAX_SPEED = METERS_PER_ROTATION*100; // motor rotates at ~ 100 rot/sec at free speed
    public static final double SPEED_LOWER_LIMIT = 0.0;
    public static final double SPEED_UPPER_LIMIT = 0.0;
    
    public static final double POSITION_LOWER_LIMIT = 0.01;
    public static final double POSITION_UPPER_LIMIT = 0.71;
    
    public static final int MOTOR_STATOR_LIMIT = 60; // needs to be tuned

    // not tuned
    public static double kS = 0;
    public static double kG = 0.17;
    public static double kV = 3; 
    public static double kA = 2;

    public static double kP = 100; 
    public static double kI = 0;
    public static double kD = 0; 

    public static double kMaxV = 2;
    public static double kMaxA = 1; 

    public static final double MANUAL_MOVE_MOTOR_SPEED = 1.5;

    public static final double MAX_POSITION_TOLERANCE = 0.005;

    public static final double MIN_POS_ARM_CLEAR = 0.4; // needs to be measured 
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
    public static double kV = 0.2;
    public static double kA = 0.5;

    public static double kP = 50;
    public static double kI = 0;
    public static double kD = 0;
    
    public static double kMaxV = 2;
    public static double kMaxA = 2.5; 

    public static final double MANUAL_MOVE_MOTOR_SPEED = 3.0;

    public static final double MAX_POSITION_TOLERANCE = 0.025; // equal to 1 degree

    public static final double MIN_POS_ELEVATOR_CLEAR = -0.4; // needs to be measured
  }

  public static class CollarConstants {
    public static final int COLLAR_MOTOR_ID = 23; // placeholder
    public static final int MOTOR_STATOR_LIMIT = 20; // needs to be tuned

    public static final int COUNTS_PER_REV = 42; // may need to be updated

    public static final double METERS_PER_ROTATION = 0.0; 
    public static final double MAX_SPEED = 0.0; 
    public static final double SPEED_LOWER_LIMIT = 0.0;
    public static final double SPEED_UPPER_LIMIT = 0.0;

    public static final double OUTTAKE_MOTOR_SPEED = 8.0;
    public static final double INTAKE_MOTOR_SPEED = 8.0;
    public static final double INTAKE_SECONDARY_MOTOR_SPEED = 1.0;

    public static final double BACKWARDS_MOTOR_SPEED = 0;

    public static final double INTAKE_STOP_OFFSET = 0; // in seconds
  }

  public static class RampConstants {

  }
  
  public static class LEDConstants {
    public static final int KPORT = 0;
    public static final int KLENGTH = 30;

    public static final double BLINK_TIME = 1; // in seconds for after intake/outtake
  }

  public static class AutoAlignConstants {
    public static final double REEF_OFFSET_RIGHT = Units.inchesToMeters(1);
    public static final double REEF_OFFSET_LEFT = Units.inchesToMeters(12);

    public static final double kMaxV = 1; // to be tuned
    public static final double kMaxA = 1; // to be tuned
  }

  public static class VisionConstants {
    public static final String CAMERA_NAME = "Arducam_OV9281_USB_Camera";  

    public static final double ROBOT_TO_CAM_X = RobotContainer.name == RobotName.DYNAMENE ? Units.inchesToMeters(-12.5) : 0.31115 ; // in meters from center of robot 
    public static final double ROBOT_TO_CAM_Y = RobotContainer.name == RobotName.DYNAMENE ? Units.inchesToMeters(-0.5) : -0.0508; // in meters from center of robot 
    public static final double ROBOT_TO_CAM_Z = RobotContainer.name == RobotName.DYNAMENE ? Units.inchesToMeters(13.5) : 0.1397; // in meters from the floor?
    
    public static final Translation2d ROBOT_TO_CAM = 
      new Translation2d(ROBOT_TO_CAM_X, ROBOT_TO_CAM_Y); // in meters from center of robot to 2x4 camera mount

    public static final Transform3d ROBOT_TO_CAM_3D = 
      new Transform3d(new Translation3d(ROBOT_TO_CAM_X, ROBOT_TO_CAM_Y, ROBOT_TO_CAM_Z), new Rotation3d(0,0,0)); // in meters from center of robot to 2x4 camera mount
    
    public static final double CAMERA_HEIGHT = 0.0951738; // in meters from floor to camera center
    public static final double CAMERA_PITCH = 0; // in radians, bogus
    public static final double TARGET_HEIGHT = 0.3048; // in meters to the middle of the apriltag on reef
    public static final double TARGET_PITCH = 0; 
  }

  public static class LaserCanConstants {
    public static final int LASER_CAN_RAMP_OUT_ID = 14;
    public static final int LASER_CAN_RAMP_IN_ID = 15;

    public static final double MIN_CORAL_SEEN_DISTANCE_RAMP_OUT = 10; // in mm
    public static final double MIN_CORAL_SEEN_DISTANCE_RAMP_IN = 300; // in mm

  }

  public static class RobotStateConstants {
    public static final double C1_ELEVATOR_POS = ElevatorConstants.INITIAL_POSITION;
    public static final double C1_ARM_POS = ArmConstants.INITIAL_POSITION;
    public static final double C2_ELEVATOR_POS = 0.36;
    public static final double C2_ARM_POS = 0.354;
    public static final double C3_ELEVATOR_POS = 0.71;
    public static final double C3_ARM_POS = 0.406;
    public static final double C4_ELEVATOR_POS = 0.71;
    public static final double C4_ARM_POS = -3.25;
    public static final double A2_ELEVATOR_POS = Math.toRadians(0.0);
    public static final double A2_ARM_POS = Math.toRadians(0.0);
    public static final double A3_ELEVATOR_POS = Math.toRadians(0.0);
    public static final double A3_ARM_POS = Math.toRadians(0.0);
    public static final double IN_ELEVATOR_POS = 0;
    public static final double IN_ARM_POS = -0.378;
  }
}
