package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.CanBridge;

import au.grapplerobotics.ConfigurationFailedException;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;


public class IntakeSubsystem extends SubsystemBase {
  private LaserCan lc;

  public IntakeSubsystem() {
    CanBridge.runTCP(); // allow grapplehook to communicate w/ lasercan:)
    lc = new LaserCan(IntakeConstants.LASER_CAN_ID);
  }

  //to-do: smartdashboard print these values instead of system.out.print.ln
  //use grapple hook; test it out; connect; set the CAN id
  @Override
  public void periodic() {
    LaserCan.Measurement measurement = lc.getMeasurement();
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      SmartDashboard.putNumber("LaserCAN Distance", measurement.distance_mm);
    } else {
      SmartDashboard.putNumber("LaserCAN Distance", 0);
      // System.out.println("Oh no! The target is out of range, or we can't get a reliable measurement!");
      // You can still use distance_mm in here, if you're ok tolerating a clamped value or an unreliable measurement.
    }
  }
}
