package frc.robot.subsystems.collar;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.CanBridge;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.LaserCanConstants;


public class RampSensorSubsystem extends SubsystemBase {
  private LaserCan LCRampOut;
  private LaserCan LCRampIn;

  private LaserCan.Measurement measurementRampOut;
  private LaserCan.Measurement measurementRampIn;


  public RampSensorSubsystem() {
    CanBridge.runTCP(); // allow grapplehook to communicate w/ lasercan:)
    LCRampOut = new LaserCan(LaserCanConstants.LASER_CAN_RAMP_OUT_ID);
    LCRampIn = new LaserCan(LaserCanConstants.LASER_CAN_RAMP_IN_ID);

  }

  public Trigger coralSeenAfterRampTrigger() {
    return new Trigger(() -> measurementRampOut != null && measurementRampOut.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT && measurementRampOut.distance_mm < LaserCanConstants.MIN_CORAL_SEEN_DISTANCE_RAMP_OUT);
  }

  public boolean coralSeenAfterRamp() {
    return measurementRampOut != null && 
      measurementRampOut.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT && 
      measurementRampOut.distance_mm < LaserCanConstants.MIN_CORAL_SEEN_DISTANCE_RAMP_OUT;
  }
  
  public Trigger coralSeenInRampTrigger() {
    return new Trigger(() -> coralSeenInRamp());
  }

  public boolean coralSeenInRamp() {
    return measurementRampIn != null && 
      measurementRampIn.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT && 
      measurementRampIn.distance_mm < LaserCanConstants.MIN_CORAL_SEEN_DISTANCE_RAMP_IN;
  }

  @Override
  public void periodic() {
    measurementRampOut = LCRampOut.getMeasurement();
    if (measurementRampOut != null && measurementRampOut.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      SmartDashboard.putNumber("rampSensors/LaserCAN Ramp Out Distance", measurementRampOut.distance_mm);
    } else {
      SmartDashboard.putNumber("rampSensors/LaserCAN Ramp Out Distance", 0);
      // You can still use distance_mm in here, if you're ok tolerating a clamped value or an unreliable measurementRampOMeasurement.
    }

    measurementRampIn = LCRampIn.getMeasurement();
    if (measurementRampIn != null && measurementRampIn.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      SmartDashboard.putNumber("rampSensors/LaserCAN Ramp In Distance", measurementRampIn.distance_mm);
    } else {
      SmartDashboard.putNumber("rampSensors/LaserCAN Ramp In Distance", 0);
      // You can still use distance_mm in here, if you're ok tolerating a clamped value or an unreliable measurementRampOMeasurement.
    }

    SmartDashboard.putBoolean("rampSensors/coralSeenAfterRamp", coralSeenAfterRamp());
    SmartDashboard.putBoolean("rampSensors/coralSeenInRamp", coralSeenInRamp());

  }
}
