// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;


import java.nio.file.SecureDirectoryStream;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.fasterxml.jackson.databind.type.PlaceholderForType;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LEDSubsystem;

public class RobotContainer {

  record JoystickVals(double x, double y) { }

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12VoltsMps desired top speed *0.3 for pid tuning 9/15
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driverJoystick = new CommandXboxController(OperatorConstants.kDriverControllerPort); // driver joystick
  private final CommandXboxController testJoystick = new CommandXboxController(OperatorConstants.kTestControllerPort); // test joystick

  
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(); // My drivetrain
  private final LEDSubsystem m_ledSubsystem = new LEDSubsystem(); 


  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.Velocity); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.FieldCentricFacingAngle snapToAngle = new SwerveRequest.FieldCentricFacingAngle().withDriveRequestType(DriveRequestType.Velocity).withVelocityX(0).withVelocityY(0);

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final SendableChooser<Command> m_autoChooser; // sendable chooser that holds the autos

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
      drivetrain.applyRequest(() -> {
        JoystickVals shapedValues = inputShape(driverJoystick.getLeftX(), driverJoystick.getLeftY());
        return drive.withVelocityX(-shapedValues.y() * MaxSpeed) // Drive forward with negative Y (forward)
          .withVelocityY(-shapedValues.x() * MaxSpeed) // Drive left with negative X (left)
          .withRotationalRate(-driverJoystick.getRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
      }));

    snapToAngle.HeadingController = new PhoenixPIDController(DrivetrainConstants.ROBOT_ROTATION_P, DrivetrainConstants.ROBOT_ROTATION_I, DrivetrainConstants.ROBOT_ROTATION_D);
    snapToAngle.HeadingController.enableContinuousInput(0, Math.PI * 2);
    //set buttons to LED lights
    // a to flash yellow
    driverJoystick.pov(0).whileTrue(m_ledSubsystem.runSolidYellow());
    driverJoystick.pov(180).whileTrue(m_ledSubsystem.runSolidBlue());

    // snap to angle
    driverJoystick.y().whileTrue(drivetrain.applyRequest(() -> snapToAngle.withTargetDirection(Rotation2d.fromDegrees(0))));
    driverJoystick.x().whileTrue(drivetrain.applyRequest(() -> snapToAngle.withTargetDirection(Rotation2d.fromDegrees(90))));
    driverJoystick.a().whileTrue(drivetrain.applyRequest(() -> snapToAngle.withTargetDirection(Rotation2d.fromDegrees(180))));
    driverJoystick.b().whileTrue(drivetrain.applyRequest(() -> snapToAngle.withTargetDirection(Rotation2d.fromDegrees(270))));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    testJoystick.leftBumper().and(testJoystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    testJoystick.leftBumper().and(testJoystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    testJoystick.rightBumper().and(testJoystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    testJoystick.rightBumper().and(testJoystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    driverJoystick.leftBumper().whileTrue(drivetrain.applyRequest(() -> brake));
    driverJoystick.rightBumper().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driverJoystick.getLeftY(), -driverJoystick.getLeftX()))));

    // reset the field-centric heading on left trigger press
    driverJoystick.leftTrigger().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    if (Utils.isSimulation()) {
      drivetrain.resetPose(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    DataLogManager.start(); // log networktable 
    DriverStation.startDataLog(DataLogManager.getLog()); // log ds state, joystick data
    DriverStation.silenceJoystickConnectionWarning(true); // turn off unplugged joystick errors 

    SignalLogger.setPath("/home/lvuser/logs/");
    SignalLogger.enableAutoLogging(false);
    // SignalLogger.start();
    

  
    // Build an auto chooser with all the PathPlanner autos. Uses Commands.none() as the default option.
    // To set a different default auto, put its name (as a String) below as a parameter
    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_autoChooser);
    for (String autoName : AutoBuilder.getAllAutoNames()) {
    }

    // Creating the tab for auto chooser in shuffleboard (under tab named "Comp HUD")
    ShuffleboardTab compTab = Shuffleboard.getTab("Comp HUD");
    compTab.add("Auto Chooser", m_autoChooser).withSize(3, 2);

    configureBindings();

  }

  // set all drivetrain motors to brake mode
  public void setBrakeDrivetrain() {
    drivetrain.setBrake(true);
  }

  // set all drivetrain motors to coast mode
  public void setCoastDrivetrain() {
    drivetrain.setBrake(false);
  }

  private JoystickVals inputShape(double x, double y) {
    double hypot = Math.hypot(x, y);
    double deadbandedValue = MathUtil.applyDeadband(hypot, OperatorConstants.JOYSTICK_DEADBAND);
  
    double scaleFactor = deadbandedValue * Math.abs(deadbandedValue) / hypot;

    JoystickVals output = new JoystickVals(x * scaleFactor, y * scaleFactor);

    return output;

  }
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

}
