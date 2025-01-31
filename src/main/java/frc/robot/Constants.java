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
    public static final int kTestControllerPort = 2;

    public static final double JOYSTICK_DEADBAND = 0.1;
  }

  public static class DrivetrainConstants {
    // Not tuned
    public static final double ROBOT_ROTATION_P = 5;
    public static final double ROBOT_ROTATION_I = 0;
    public static final double ROBOT_ROTATION_D = 0;
    public static final double ROBOT_POSITION_P = 5;
    public static final double ROBOT_POSITION_I = 0;
    public static final double ROBOT_POSITION_D = 0;
  }
  
  public static class LEDConstants {
    public static final int KPORT = 0;
    public static final int KLENGTH = 30;
  }

  public static class VisionConstants {
    public static final String CAMERA_NAME = "Arducam_OV9782_USB_Camera";  

    public static final Translation2d ROBOT_TO_CAM = 
      new Translation2d(0.3048, 0); // in meters from center of robot to 2x4 camera mount
    
    public static final double CAMERA_HEIGHT = 0.0951738; // in meters from floor to camera center
    public static final double CAMERA_PITCH = 0; // in radians, bogus
    public static final double TARGET_HEIGHT = 0.3048; // in meters to the middle of the apriltag on reef
  }

  public static class IntakeConstants {
    public static final int LASER_CAN_ID = 14;
  }
}
