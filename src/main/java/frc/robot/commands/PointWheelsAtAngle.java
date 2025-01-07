// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** A command that changes the angles of the wheels */
public class PointWheelsAtAngle extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final CommandSwerveDrivetrain m_drivetrain;
  private final double m_degrees;

  /**
   * Creates a new PointWheelsAtAngle.
   *
   * @param drivetrain The subsystem used by this command.
   */
  public PointWheelsAtAngle(CommandSwerveDrivetrain drivetrain, double degrees) {
    m_degrees = degrees;
    m_drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SwerveRequest.PointWheelsAt wheelRequest = new SwerveRequest.PointWheelsAt();
    m_drivetrain.setControl(wheelRequest.withModuleDirection(Rotation2d.fromDegrees(m_degrees)));
    // SmartDashboard.putNumber("Hood Encoder", m_drivetrain.getModule(0).getTargetState());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
