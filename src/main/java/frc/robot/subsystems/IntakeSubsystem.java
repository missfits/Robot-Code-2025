package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.CanBridge;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;


public class IntakeSubsystem extends SubsystemBase {
  private LaserCan lc;

  public IntakeSubsystem() {
    CanBridge.runTCP(); // allow grapplehook to communicate w/ lasercan:)
    lc = new LaserCan(IntakeConstants.LASER_CAN_ID);
  }

  @Override
  public void periodic() {
    LaserCan.Measurement measurement = lc.getMeasurement();
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      SmartDashboard.putNumber("LaserCAN Distance", measurement.distance_mm);
    } else {
      SmartDashboard.putNumber("LaserCAN Distance", 0);
      // You can still use distance_mm in here, if you're ok tolerating a clamped value or an unreliable measurement.
    }
  }
}
