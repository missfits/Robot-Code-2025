package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private LaserCan lc;

  public IntakeSubsystem() {
    lc = new LaserCan(0);
    // Optionally initialise the settings of the LaserCAN, if you haven't already done so in GrappleHook
    try {
      lc.setRangingMode(LaserCan.RangingMode.SHORT);
      lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }
  }

  //to-do: smartdashboard print these values instead of system.out.print.ln
  //use grapple hook; test it out; connect; set the CAN id
  @Override
  public void periodic() {
    LaserCan.Measurement measurement = lc.getMeasurement();
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      System.out.println("The target is " + measurement.distance_mm + "mm away!");
    } else {
      System.out.println("Oh no! The target is out of range, or we can't get a reliable measurement!");
      // You can still use distance_mm in here, if you're ok tolerating a clamped value or an unreliable measurement.
    }
  }
}
