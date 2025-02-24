// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.fasterxml.jackson.databind.type.PlaceholderForType;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveToReefCommand;
import frc.robot.commands.RotateToFaceReefCommand;
import frc.robot.generated.TunerConstantsCeridwen;
import frc.robot.generated.TunerConstantsDynamene;
import frc.robot.commands.DriveToReefCommand.ReefPosition;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.collar.CollarSubsystem;
import frc.robot.subsystems.lifter.ArmSubsystem;
import frc.robot.subsystems.lifter.ElevatorIOHardware;
import frc.robot.subsystems.lifter.ElevatorSubsystem;
import frc.robot.subsystems.lifter.LifterCommandFactory;
import frc.robot.subsystems.ramp.RampSubsystem;
import frc.robot.commands.AutoAlignCommand;



public class RobotContainer {

  record JoystickVals(double x, double y) { }
  
  public enum RobotName {
      CERIDWEN,
      DYNAMENE
  }

  public static RobotName name = RobotName.DYNAMENE;

  private RobotState currentState = RobotState.INTAKE;
  private RobotState nextState = RobotState.INTAKE;

  private double MaxSpeed = TunerConstantsDynamene.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12VoltsMps desired top speed *0.3 for pid tuning 9/15
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driverJoystick = new CommandXboxController(OperatorConstants.kDriverControllerPort); // driver joystick
  private final CommandXboxController copilotJoystick = new CommandXboxController(OperatorConstants.kCopilotControllerPort); // copilot joystick
  private final CommandXboxController testJoystick = new CommandXboxController(OperatorConstants.kTestControllerPort); // test joystick

  
  private final CommandSwerveDrivetrain drivetrain = 
   name == RobotName.DYNAMENE ? TunerConstantsDynamene.createDrivetrain() : TunerConstantsCeridwen.createDrivetrain(); // My drivetrain

  private final LEDSubsystem m_ledSubsystem = new LEDSubsystem(); 
  private final VisionSubsystem m_vision = new VisionSubsystem(drivetrain.getPigeon2());
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem(); 
  private final CollarSubsystem m_collar = new CollarSubsystem();
  private final RampSubsystem m_ramp = new RampSubsystem();
  private final LifterCommandFactory m_lifter = new LifterCommandFactory();
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();

  private final ArmSubsystem m_arm = new ArmSubsystem();


  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.Velocity); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle().withDriveRequestType(DriveRequestType.Velocity);

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final SendableChooser<Command> m_autoChooser; // sendable chooser that holds the autos

  private final Field2d m_estPoseField = new Field2d();
  private final Field2d m_actualField = new Field2d();


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
    driverJoystick.leftTrigger().onTrue(drivetrain.runOnce(() -> drivetrain.setNewPose(new Pose2d(0,0,new Rotation2d(0)))));

    driverJoystick.rightTrigger().whileTrue(new RotateToFaceReefCommand(drivetrain, m_vision));
    
    // "temporary" for testing. moves to the RIGHT side. only press after running rotatetofacereef (right trigger)
    driverJoystick.rightBumper().whileTrue(new DriveToReefCommand(drivetrain, m_vision, ReefPosition.RIGHT)); 

    // "temporary" for testing. moves to the LEFT side. only press after running rotatetofacereef (right trigger)
    driverJoystick.leftBumper().whileTrue(new DriveToReefCommand(drivetrain, m_vision, ReefPosition.LEFT)); 

    // // move lifter to next position 
    // driverJoystick.leftBumper().onTrue(
    //   new ParallelCommandGroup(
    //     m_lifter.getCommand(nextState),
    //     new InstantCommand(() -> {currentState = nextState; nextState = RobotState.INTAKE;})));


    // // outtake from collar, then move lifter to the default position
    // driverJoystick.rightBumper().onTrue(
    //   m_collar.getCommand(currentState)

    //   .andThen(new ParallelCommandGroup(
    //     // move the lifter to the intake (default) position 
    //     new InstantCommand(() -> {currentState = nextState; nextState = RobotState.INTAKE;}),
    //     m_lifter.getCommand(currentState))));


    // set next state, change LED colors accordingly 
    // copilotJoystick.leftTrigger().onTrue(
    //   new ParallelCommandGroup(
    //   new InstantCommand(() -> {nextState = RobotState.L4_CORAL;}),
    //   m_ledSubsystem.runSolidRed())); 

    // copilotJoystick.rightTrigger().onTrue(
    //   new ParallelCommandGroup(
    //   new InstantCommand(() -> {nextState = RobotState.L3_CORAL;}),
    //   m_ledSubsystem.runSolidOrange())); 

    // copilotJoystick.leftBumper().onTrue(
    //   new ParallelCommandGroup(
    //   new InstantCommand(() -> {nextState = RobotState.L2_CORAL;}),
    //   m_ledSubsystem.runSolidYellow())); 

    // copilotJoystick.rightBumper().onTrue(
    //   new ParallelCommandGroup(
    //   new InstantCommand(() -> {nextState = RobotState.L1_CORAL;}),
    //   m_ledSubsystem.runSolidWhite())); 

    // copilotJoystick.a().onTrue(
    //   new ParallelCommandGroup(
    //   new InstantCommand(() -> {nextState = RobotState.L3_ALGAE;}),
    //   m_ledSubsystem.runSolidPurple())); 

    // copilotJoystick.y().onTrue(
    //   new ParallelCommandGroup(
    //   new InstantCommand(() -> {nextState = RobotState.L2_ALGAE;}),
    //   m_ledSubsystem.runSolidPink())); 

    //open loop control testing:
    copilotJoystick.leftTrigger().whileTrue(
      m_elevator.manualMoveCommand());
    
    copilotJoystick.rightTrigger().whileTrue(
      m_arm.manualMoveCommand());

    copilotJoystick.leftBumper().whileTrue(
      m_elevator.manualMoveBackwardCommand());
    
    copilotJoystick.rightBumper().whileTrue(
      m_arm.manualMoveBackwardCommand());

    copilotJoystick.a().whileTrue(
      m_collar.runCollar());
    
    copilotJoystick.y().whileTrue(
      m_collar.runCollarBackward());

    m_collar.setDefaultCommand(m_collar.runCollarOff());
    m_elevator.setDefaultCommand(m_elevator.keepInPlaceCommand());
    m_arm.setDefaultCommand(m_arm.keepInPlaceCommand());


    
    // run command runSolidGreen continuously if robot isWithinTarget()
    m_vision.isWithinTargetTrigger(() -> drivetrain.getState().Pose).whileTrue(m_ledSubsystem.runSolidGreen());

    //set buttons to LED lights
    // a to flash yellow
    testJoystick.pov(0).whileTrue(m_ledSubsystem.runSolidYellow());
    testJoystick.pov(180).whileTrue(m_ledSubsystem.runSolidBlue());

    testJoystick.leftTrigger().and(testJoystick.a()).onTrue(m_lifter.moveToCommand(RobotState.L1_CORAL));
    testJoystick.leftTrigger().and(testJoystick.x()).onTrue(m_lifter.moveToCommand(RobotState.L2_CORAL));
    testJoystick.leftTrigger().and(testJoystick.b()).onTrue(m_lifter.moveToCommand(RobotState.L3_CORAL));
    testJoystick.leftTrigger().and(testJoystick.y()).onTrue(m_lifter.moveToCommand(RobotState.L4_CORAL));
    testJoystick.rightTrigger().and(testJoystick.x()).onTrue(m_lifter.moveToCommand(RobotState.INTAKE));
    testJoystick.rightTrigger().and(testJoystick.a()).whileTrue(m_collar.runCollar());
    testJoystick.rightTrigger().and(testJoystick.y()).whileTrue(m_collar.runCollarBackward());
    

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

  private void resetControllerConstantsSmartDashboard() {
    ArmConstants.kS = SmartDashboard.getNumber("arm constants/kS", 0);
    ArmConstants.kG = SmartDashboard.getNumber("arm constants/kG", 0);
    ArmConstants.kV = SmartDashboard.getNumber("arm constants/kV", 0);
    ArmConstants.kA = SmartDashboard.getNumber("arm constants/kA", 0);
    ArmConstants.kP = SmartDashboard.getNumber("arm constants/kP", 0);
    ArmConstants.kI = SmartDashboard.getNumber("arm constants/kI", 0);
    ArmConstants.kD = SmartDashboard.getNumber("arm constants/kD", 0);
    ArmConstants.kMaxV = SmartDashboard.getNumber("arm constants/kMaxV", 0);
    ArmConstants.kMaxA = SmartDashboard.getNumber("arm constants/kMaxA", 0);

    ElevatorConstants.kS = SmartDashboard.getNumber("elevator constants/kS", 0);
    ElevatorConstants.kG = SmartDashboard.getNumber("elevator constants/kG", 0);
    ElevatorConstants.kV = SmartDashboard.getNumber("elevator constants/kV", 0);
    ElevatorConstants.kA = SmartDashboard.getNumber("elevator constants/kA", 0);
    ElevatorConstants.kP = SmartDashboard.getNumber("elevator constants/kP", 0);
    ElevatorConstants.kI = SmartDashboard.getNumber("elevator constants/kI", 0);
    ElevatorConstants.kD = SmartDashboard.getNumber("elevator constants/kD", 0);
    ElevatorConstants.kMaxV = SmartDashboard.getNumber("elevator constants/kMaxV", 0);
    ElevatorConstants.kMaxA = SmartDashboard.getNumber("elevator constants/kMaxA", 0);
    
    m_lifter.resetControllers();
  }

  public RobotContainer() {
    DataLogManager.start(); // log networktable 
    DriverStation.startDataLog(DataLogManager.getLog()); // log ds state, joystick data to /u/logs w/ usb stick, or home/lvuser/logs without. 
    DriverStation.silenceJoystickConnectionWarning(true); // turn off unplugged joystick errors 

    SignalLogger.enableAutoLogging(false);
    // SignalLogger.start();
    
    SmartDashboard.putData("est pose field", m_estPoseField);
    SmartDashboard.putData("Actual Field", m_actualField);
  
    // Build an auto chooser with all the PathPlanner autos. Uses Commands.none() as the default option.
    // To set a different default auto, put its name (as a String) below as a parameter
    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_autoChooser);
    for (String autoName : AutoBuilder.getAllAutoNames()) {
    }

    // Creating the tab for auto chooser in shuffleboard (under tab named "Comp HUD")
    ShuffleboardTab compTab = Shuffleboard.getTab("Comp HUD");
    compTab.add("Auto Chooser", m_autoChooser).withSize(3, 2);

    // setup adjustable values in smartdashboard (set them to existing values if they exist)
    SmartDashboard.putNumber("arm constants/kS", SmartDashboard.getNumber("arm constants/kS", ArmConstants.kS));
    SmartDashboard.putNumber("arm constants/kG", SmartDashboard.getNumber("arm constants/kG", ArmConstants.kG));
    SmartDashboard.putNumber("arm constants/kV", SmartDashboard.getNumber("arm constants/kV", ArmConstants.kV));
    SmartDashboard.putNumber("arm constants/kA", SmartDashboard.getNumber("arm constants/kA", ArmConstants.kA));
    SmartDashboard.putNumber("arm constants/kP", SmartDashboard.getNumber("arm constants/kP", ArmConstants.kP));
    SmartDashboard.putNumber("arm constants/kI", SmartDashboard.getNumber("arm constants/kI", ArmConstants.kI));
    SmartDashboard.putNumber("arm constants/kD", SmartDashboard.getNumber("arm constants/kD", ArmConstants.kD));
    SmartDashboard.putNumber("arm constants/kMaxV", SmartDashboard.getNumber("arm constants/kMaxV", ArmConstants.kMaxV));
    SmartDashboard.putNumber("arm constants/kMaxA", SmartDashboard.getNumber("arm constants/kMaxA", ArmConstants.kMaxA));

    SmartDashboard.putNumber("elevator constants/kS", SmartDashboard.getNumber("elevator constants/kS", ElevatorConstants.kS));
    SmartDashboard.putNumber("elevator constants/kG", SmartDashboard.getNumber("elevator constants/kG", ElevatorConstants.kG));
    SmartDashboard.putNumber("elevator constants/kV", SmartDashboard.getNumber("elevator constants/kV", ElevatorConstants.kV));
    SmartDashboard.putNumber("elevator constants/kA", SmartDashboard.getNumber("elevator constants/kA", ElevatorConstants.kA));
    SmartDashboard.putNumber("elevator constants/kP", SmartDashboard.getNumber("elevator constants/kP", ElevatorConstants.kP));
    SmartDashboard.putNumber("elevator constants/kI", SmartDashboard.getNumber("elevator constants/kI", ElevatorConstants.kI));
    SmartDashboard.putNumber("elevator constants/kD", SmartDashboard.getNumber("elevator constants/kD", ElevatorConstants.kD));
    SmartDashboard.putNumber("elevator constants/kMaxV", SmartDashboard.getNumber("elevator constants/kMaxV", ElevatorConstants.kMaxV));
    SmartDashboard.putNumber("elevator constants/kMaxA", SmartDashboard.getNumber("elevator constants/kMaxA", ElevatorConstants.kMaxA));


    configureBindings();

  }

  // set motors to appropriate neutral modes for an enabled robot
  public void setEnabledNeutralMode() {
    drivetrain.setBrake(true);
    m_lifter.setEnabledNeutralMode();
  }

  // set motors to appropriate neutral modes for an disabled robot
  public void setDisabledNeutralMode() {
    drivetrain.setBrake(false);
    m_lifter.setDisabledNeutralMode();

  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

  public void updatePoseEst() {

    EstimatedRobotPose estimatedRobotPose = m_vision.getEstimatedRobotPose();
    if (estimatedRobotPose != null) {
      Pose2d estPose2d = estimatedRobotPose.estimatedPose.toPose2d();
    
      // check if new estimated pose and previous pose are less than 2 meters apart
      if (estPose2d.getTranslation().getDistance(drivetrain.getState().Pose.getTranslation()) < 2) {
        drivetrain.poseEstimator.addVisionMeasurement(estPose2d, estimatedRobotPose.timestampSeconds);

        m_estPoseField.setRobotPose(estPose2d);
      }
    }
    

    m_actualField.setRobotPose(drivetrain.getState().Pose);
    drivetrain.updatePoseWithPoseEst();
  }

}
