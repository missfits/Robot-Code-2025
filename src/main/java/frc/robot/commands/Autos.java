// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  public static SequentialCommandGroup wheelRotationTest(CommandSwerveDrivetrain drivetrain) {
    return new SequentialCommandGroup(
      // changed to only 180 both ways 9/15 pid tuning
      new PointWheelsAtAngle(drivetrain, 0).withTimeout(2),
      new PointWheelsAtAngle(drivetrain, 60).withTimeout(2),
      new PointWheelsAtAngle(drivetrain, 120).withTimeout(2),
      new PointWheelsAtAngle(drivetrain, 180).withTimeout(2),
      // new PointWheelsAtAngle(drivetrain, 240).withTimeout(2),
      // new PointWheelsAtAngle(drivetrain, 300).withTimeout(2),
      // new PointWheelsAtAngle(drivetrain, 360).withTimeout(2),
      new PointWheelsAtAngle(drivetrain, -60).withTimeout(2),
      new PointWheelsAtAngle(drivetrain, -120).withTimeout(2),
      new PointWheelsAtAngle(drivetrain, -180).withTimeout(2)
      // new PointWheelsAtAngle(drivetrain, -240).withTimeout(2),
      // new PointWheelsAtAngle(drivetrain, -300).withTimeout(2),
      // new PointWheelsAtAngle(drivetrain, -360).withTimeout(2)

    );
  }
}
