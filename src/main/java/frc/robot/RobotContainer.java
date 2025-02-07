// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;


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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RobotStateConstants;
import frc.robot.commands.Autos;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.collar.CollarSubsystem;
import frc.robot.subsystems.lifter.LifterSubsystem;
import frc.robot.subsystems.ramp.RampSubsystem;
import frc.robot.commands.AutoAlignCommand;


public class RobotContainer {

  record JoystickVals(double x, double y) { }

  public enum RobotState { 
    L1_CORAL(RobotStateConstants.C1_ELEVATOR_POS, RobotStateConstants.C1_ARM_POS), 
    L2_CORAL(RobotStateConstants.C2_ELEVATOR_POS, RobotStateConstants.C2_ARM_POS), 
    L3_CORAL(RobotStateConstants.C3_ELEVATOR_POS, RobotStateConstants.C3_ARM_POS), 
    L4_CORAL(RobotStateConstants.C4_ELEVATOR_POS, RobotStateConstants.C4_ARM_POS), 
    L2_ALGAE(RobotStateConstants.A2_ELEVATOR_POS, RobotStateConstants.A2_ARM_POS), 
    L3_ALGAE(RobotStateConstants.A3_ELEVATOR_POS, RobotStateConstants.A3_ARM_POS), 
    INTAKE(RobotStateConstants.IN_ELEVATOR_POS, RobotStateConstants.IN_ARM_POS);

    private final double elevatorPos;
    private final double armPos;

    // constructor
    private RobotState(double ep, double ap) {
      elevatorPos = ep;
      armPos = ap;
    }

    // getters
    public double getElevatorPos() {
      return elevatorPos;
    }

    public double getArmPos() {
      return armPos;
    }
  } 

  private RobotState currentState = RobotState.INTAKE;
  private RobotState nextState = RobotState.INTAKE;

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12VoltsMps desired top speed *0.3 for pid tuning 9/15
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driverJoystick = new CommandXboxController(OperatorConstants.kDriverControllerPort); // driver joystick
  private final CommandXboxController copilotJoystick = new CommandXboxController(OperatorConstants.kCopilotControllerPort); // copilot joystick
  private final CommandXboxController testJoystick = new CommandXboxController(OperatorConstants.kTestControllerPort); // test joystick

  
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(); // My drivetrain
  private final LEDSubsystem m_ledSubsystem = new LEDSubsystem(); 
  private final VisionSubsystem m_vision = new VisionSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem(); 
  private final CollarSubsystem m_collar = new CollarSubsystem();
  private final LifterSubsystem m_lifter = new LifterSubsystem();
  private final RampSubsystem m_ramp = new RampSubsystem();

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.Velocity); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle().withDriveRequestType(DriveRequestType.Velocity);

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final SendableChooser<Command> m_autoChooser; // sendable chooser that holds the autos

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
      drivetrain.getCommandFromRequest(() -> {
        JoystickVals shapedValues = Controls.adjustInputs(driverJoystick.getLeftX(), driverJoystick.getLeftY(), driverJoystick.rightTrigger().getAsBoolean());
        return drive.withVelocityX(-shapedValues.y() * MaxSpeed) // Drive forward with negative Y (forward)
          .withVelocityY(-shapedValues.x() * MaxSpeed) // Drive left with negative X (left)
          .withRotationalRate(-driverJoystick.getRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
      }));

    driveFacingAngle.HeadingController = new PhoenixPIDController(DrivetrainConstants.ROBOT_ROTATION_P, DrivetrainConstants.ROBOT_ROTATION_I, DrivetrainConstants.ROBOT_ROTATION_D);
    driveFacingAngle.HeadingController.enableContinuousInput(0, Math.PI * 2);
   
    // drive facing angle buttons
    // can be pressed alone for rotation or pressed with joystick input
    driverJoystick.y().whileTrue(drivetrain.getCommandFromRequest(() -> {
      JoystickVals shapedValues = Controls.adjustInputs(driverJoystick.getLeftX(), driverJoystick.getLeftY(), driverJoystick.rightTrigger().getAsBoolean());
      return driveFacingAngle.withVelocityX(-shapedValues.y() * MaxSpeed) // Drive forward with negative Y (forward)
        .withVelocityY(-shapedValues.x() * MaxSpeed) // Drive left with negative X (left)
        .withTargetDirection(Rotation2d.fromDegrees(0));
    }));
    driverJoystick.x().whileTrue(drivetrain.getCommandFromRequest(() -> {
      JoystickVals shapedValues = Controls.adjustInputs(driverJoystick.getLeftX(), driverJoystick.getLeftY(), driverJoystick.rightTrigger().getAsBoolean());
      return driveFacingAngle.withVelocityX(-shapedValues.y() * MaxSpeed) // Drive forward with negative Y (forward)
        .withVelocityY(-shapedValues.x() * MaxSpeed) // Drive left with negative X (left)
        .withTargetDirection(Rotation2d.fromDegrees(90));
    }));
    driverJoystick.a().whileTrue(drivetrain.getCommandFromRequest(() -> {
      JoystickVals shapedValues = Controls.adjustInputs(driverJoystick.getLeftX(), driverJoystick.getLeftY(), driverJoystick.rightTrigger().getAsBoolean());
      return driveFacingAngle.withVelocityX(-shapedValues.y() * MaxSpeed) // Drive forward with negative Y (forward)
        .withVelocityY(-shapedValues.x() * MaxSpeed) // Drive left with negative X (left)
        .withTargetDirection(Rotation2d.fromDegrees(180));
    }));
    driverJoystick.b().whileTrue(drivetrain.getCommandFromRequest(() -> {
      JoystickVals shapedValues = Controls.adjustInputs(driverJoystick.getLeftX(), driverJoystick.getLeftY(), driverJoystick.rightTrigger().getAsBoolean());
      return driveFacingAngle.withVelocityX(-shapedValues.y() * MaxSpeed) // Drive forward with negative Y (forward)
        .withVelocityY(-shapedValues.x() * MaxSpeed) // Drive left with negative X (left)
        .withTargetDirection(Rotation2d.fromDegrees(270));
    }));

    // reset the field-centric heading on left trigger press
    driverJoystick.leftTrigger().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    driverJoystick.rightTrigger().whileTrue(new AutoAlignCommand(drivetrain, m_vision));

    driverJoystick.leftBumper().onTrue(
      new ParallelCommandGroup(
        m_lifter.getCommand(nextState),
        new InstantCommand(() -> {currentState = nextState; nextState = RobotState.INTAKE;})));

    driverJoystick.rightBumper().onTrue(m_collar.getCommand(currentState));


    copilotJoystick.leftTrigger().onTrue(
      new InstantCommand(() -> {nextState = RobotState.L4_CORAL;}));
    copilotJoystick.rightTrigger().onTrue(
      new InstantCommand(() -> {nextState = RobotState.L3_CORAL;}));
    copilotJoystick.leftBumper().onTrue(
      new InstantCommand(() -> {nextState = RobotState.L2_CORAL;}));
    copilotJoystick.rightBumper().onTrue(
      new InstantCommand(() -> {nextState = RobotState.L1_CORAL;}));
    copilotJoystick.a().onTrue(
      new InstantCommand(() -> {nextState = RobotState.L3_ALGAE;}));
    copilotJoystick.y().onTrue(
      new InstantCommand(() -> {nextState = RobotState.L2_ALGAE;}));

    // run command runSolidGreen continuously if robot isWithinTarget()
    m_vision.isWithinTargetTrigger().whileTrue(m_ledSubsystem.runSolidGreen());

    //set buttons to LED lights
    // a to flash yellow
    testJoystick.pov(0).whileTrue(m_ledSubsystem.runSolidYellow());
    testJoystick.pov(180).whileTrue(m_ledSubsystem.runSolidBlue());

    testJoystick.rightTrigger().whileTrue(new AutoAlignCommand(drivetrain, m_vision));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    testJoystick.leftBumper().and(testJoystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    testJoystick.leftBumper().and(testJoystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    testJoystick.rightBumper().and(testJoystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    testJoystick.rightBumper().and(testJoystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    if (Utils.isSimulation()) {
      drivetrain.resetPose(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    DataLogManager.start(); // log networktable 
    DriverStation.startDataLog(DataLogManager.getLog()); // log ds state, joystick data to /u/logs w/ usb stick, or home/lvuser/logs without. 
    DriverStation.silenceJoystickConnectionWarning(true); // turn off unplugged joystick errors 

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

  // set motors to appropriate neutral modes for an enabled robot
  public void setEnabledNeutralMode() {
    drivetrain.setBrake(true);
  }

  // set motors to appropriate neutral modes for an disabled robot
  public void setDisabledNeutralMode() {
    drivetrain.setBrake(false);
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

}
