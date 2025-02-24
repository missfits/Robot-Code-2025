package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.CanBridge;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.LaserCanConstants;


public class LaserCanSubsystem extends SubsystemBase {
  private LaserCan lc;
  private LaserCan.Measurement measurement;

  public LaserCanSubsystem() {
    CanBridge.runTCP(); // allow grapplehook to communicate w/ lasercan:)
    lc = new LaserCan(LaserCanConstants.LASER_CAN_ID);
  }

  public Trigger coralSeen() {
    return new Trigger(() -> measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT && measurement.distance_mm < LaserCanConstants.MIN_CORAL_SEEN_DISTANCE);
  }

  @Override
  public void periodic() {
    measurement = lc.getMeasurement();
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      SmartDashboard.putNumber("LaserCAN Distance", measurement.distance_mm);
    } else {
      SmartDashboard.putNumber("LaserCAN Distance", 0);
      // You can still use distance_mm in here, if you're ok tolerating a clamped value or an unreliable measurement.
    }
  }
}
