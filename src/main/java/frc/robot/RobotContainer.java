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
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.fasterxml.jackson.databind.type.PlaceholderForType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CollarConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveToReefCommand;
import frc.robot.commands.RotateToFaceReefCommand;
import frc.robot.generated.TunerConstantsCeridwen;
import frc.robot.generated.TunerConstantsDynamene;
import frc.robot.commands.DriveToReefCommand.ReefPosition;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.collar.CollarSubsystem;
import frc.robot.subsystems.collar.CollarCommandFactory;
import frc.robot.subsystems.collar.RampSensorSubsystem;
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
  private CommandXboxController testJoystick = null; // test joystick; only init'd if we're not in comp mode

  
  private final CommandSwerveDrivetrain drivetrain = 
   name == RobotName.DYNAMENE ? TunerConstantsDynamene.createDrivetrain() : TunerConstantsCeridwen.createDrivetrain(); // My drivetrain

  private final LEDSubsystem m_ledSubsystem = new LEDSubsystem(); 
  private final VisionSubsystem m_vision = new VisionSubsystem(drivetrain.getPigeon2());
  private final RampSensorSubsystem m_rampSensor = new RampSensorSubsystem(); 
  private final CollarSubsystem m_collar = new CollarSubsystem();
  private final RampSubsystem m_ramp = new RampSubsystem();
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final ArmSubsystem m_arm = new ArmSubsystem();
  private final ClimberSubsystem m_climber = new ClimberSubsystem();


  private final CollarCommandFactory m_collarCommandFactory = new CollarCommandFactory(m_collar, m_rampSensor);

  private final LifterCommandFactory m_lifter = new LifterCommandFactory(m_elevator, m_arm);


  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.Velocity); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle().withDriveRequestType(DriveRequestType.Velocity).withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final SendableChooser<Command> m_autoChooser; // sendable chooser that holds the autos

  private final Field2d m_estPoseField = new Field2d();
  private final Field2d m_actualField = new Field2d();


  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
      drivetrain.getCommandFromRequest(() -> {
        JoystickVals shapedValues = Controls.adjustDrivetrainInputs(driverJoystick.getLeftX(), driverJoystick.getLeftY(), driverJoystick.rightBumper().getAsBoolean(), m_elevator.isTallTrigger().getAsBoolean());
        return drive.withVelocityX(-shapedValues.y() * MaxSpeed) // Drive forward with negative Y (forward)
          .withVelocityY(-shapedValues.x() * MaxSpeed) // Drive left with negative X (left)
          .withRotationalRate(-driverJoystick.getRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
      }));

    driveFacingAngle.HeadingController = new PhoenixPIDController(DrivetrainConstants.ROBOT_ROTATION_P, DrivetrainConstants.ROBOT_ROTATION_I, DrivetrainConstants.ROBOT_ROTATION_D);
    driveFacingAngle.HeadingController.enableContinuousInput(0, Math.PI * 2);

    // reset the field-centric heading on a button press
    driverJoystick.a().onTrue(drivetrain.runOnce(() -> drivetrain.resetRotation(new Rotation2d(DriverStation.getAlliance().equals(Alliance.Blue) ? 0 : Math.PI))));

    // reset fused vision pose estimator to vision pose on center (cross button)
    driverJoystick.povCenter().onTrue(drivetrain.runOnce(() -> drivetrain.resetFusedPose(m_vision.getEstimatedRobotPose().estimatedPose.toPose2d())));

    // auto rotate to reef command
    driverJoystick.y().whileTrue(new RotateToFaceReefCommand(drivetrain, m_vision));
  
    // moves to the RIGHT side. only press after running rotatetofacereef (right trigger)
    driverJoystick.rightTrigger().whileTrue(new DriveToReefCommand(drivetrain, m_vision, ReefPosition.RIGHT, m_ledSubsystem)); 
    driverJoystick.rightTrigger().and(drivetrain.isAutoAligned().negate()).whileTrue(m_ledSubsystem.runSolidRed()); 
    drivetrain.isAutoAligned().whileTrue(m_ledSubsystem.runGradientGreenYellow());

    // moves to the LEFT side. only press after running rotatetofacereef (right trigger)
    driverJoystick.leftTrigger().whileTrue(new DriveToReefCommand(drivetrain, m_vision, ReefPosition.LEFT, m_ledSubsystem)); 
    driverJoystick.leftTrigger().and(drivetrain.isAutoAligned().negate()).whileTrue(m_ledSubsystem.runSolidRed()); 
    
    driverJoystick.b().whileTrue(m_climber.manualMoveBackwardCommand());
    driverJoystick.x().whileTrue(m_climber.manualMoveCommand());

    // drive facing angle buttons
    // can be pressed alone for rotation or pressed with joystick input
    // snap to left coral station
    driverJoystick.x().whileTrue(drivetrain.getCommandFromRequest(() -> {
      JoystickVals shapedValues = Controls.adjustDrivetrainInputs(driverJoystick.getLeftX(), driverJoystick.getLeftY(), driverJoystick.rightBumper().getAsBoolean(), m_elevator.isTallTrigger().getAsBoolean());
      return driveFacingAngle.withVelocityX(-shapedValues.y() * MaxSpeed) // Drive forward with negative Y (forward)
        .withVelocityY(-shapedValues.x() * MaxSpeed) // Drive left with negative X (left)
        .withTargetDirection(Rotation2d.fromDegrees(-55));
    }));

    // snap to right coral station
    driverJoystick.y().whileTrue(drivetrain.getCommandFromRequest(() -> {
      JoystickVals shapedValues = Controls.adjustDrivetrainInputs(driverJoystick.getLeftX(), driverJoystick.getLeftY(), driverJoystick.rightBumper().getAsBoolean(), m_elevator.isTallTrigger().getAsBoolean());
      return driveFacingAngle.withVelocityX(-shapedValues.y() * MaxSpeed) // Drive forward with negative Y (forward)
        .withVelocityY(-shapedValues.x() * MaxSpeed) // Drive left with negative X (left)
        .withTargetDirection(Rotation2d.fromDegrees(55));
    }));

    // move to intake, start collar 
    copilotJoystick.x().and(copilotJoystick.povCenter()).onTrue(
      m_lifter.moveToCommand(RobotState.INTAKE)
    );

    copilotJoystick.x().and(copilotJoystick.povCenter()).whileTrue(
      m_collarCommandFactory.intakeCoralSequence()
    );

    // outtake from collar
    copilotJoystick.y().and(copilotJoystick.povCenter()).whileTrue(
      m_collarCommandFactory.runCollarOut()
    );

    copilotJoystick.b().whileTrue(
      m_collarCommandFactory.runCollarBackwards()
    ); 

    // set move to state
    // C4 -> rightTrigger; !pov (any), !start
    copilotJoystick.rightTrigger().and(copilotJoystick.povCenter()).and(copilotJoystick.start().negate()).onTrue(
      m_lifter.moveToCommand(RobotState.L4_CORAL)
    ); 

    // C4_WITH_CORAL -> rightTrigger + start; !pov (any)
    copilotJoystick.rightTrigger().and(copilotJoystick.povCenter()).and(copilotJoystick.start()).onTrue(
      m_lifter.moveToCommand(RobotState.L4_CORAL_WITH_CORAL)
    ); 

    // C3 -> leftTrigger; !pov (any), !start
    copilotJoystick.leftTrigger().and(copilotJoystick.povCenter()).and(copilotJoystick.start().negate()).onTrue(
      m_lifter.moveToCommand(RobotState.L3_CORAL)
    ); 

    // C3_WITH_CORAL -> leftTrigger + start; !pov (any)
    copilotJoystick.leftTrigger().and(copilotJoystick.povCenter()).and(copilotJoystick.start()).onTrue(
      m_lifter.moveToCommand(RobotState.L3_CORAL_WITH_CORAL)
    ); 

    // C2 -> rightBumper; !a, !pov (any), !start
    copilotJoystick.rightBumper().and(copilotJoystick.a().negate()).and(copilotJoystick.povCenter()).and(copilotJoystick.start().negate()).onTrue(
      m_lifter.moveToCommand(RobotState.L2_CORAL)
    ); 

    // C2_WITH_CORAL -> rightBumper + start; !a, !pov (any)
    copilotJoystick.rightBumper().and(copilotJoystick.a().negate()).and(copilotJoystick.povCenter()).and(copilotJoystick.start()).onTrue(
      m_lifter.moveToCommand(RobotState.L2_CORAL_WITH_CORAL)
    ); 

    // C1 -> leftBumper; !a, !pov (any)
    copilotJoystick.leftBumper().and(copilotJoystick.a().negate()).and(copilotJoystick.povCenter()).onTrue(
      m_lifter.moveToCommand(RobotState.L1_CORAL)
    ); 

    // A3 -> rightBumper + a; !pov (any)
    copilotJoystick.rightBumper().and(copilotJoystick.a()).and(copilotJoystick.povCenter()).onTrue(
      m_lifter.moveToCommand(RobotState.L3_ALGAE)
    ); 

    copilotJoystick.rightBumper().and(copilotJoystick.a()).and(copilotJoystick.povCenter()).whileTrue(
      m_collarCommandFactory.runCollarOut()
    ); 

    // A2 -> leftBumper + a; !pov (any)
    copilotJoystick.leftBumper().and(copilotJoystick.a()).and(copilotJoystick.povCenter()).onTrue(
      m_lifter.moveToCommand(RobotState.L2_ALGAE)
    ); 

    copilotJoystick.leftBumper().and(copilotJoystick.a()).and(copilotJoystick.povCenter()).whileTrue(
      m_collarCommandFactory.runCollarOut()
    ); 


    // lifter backup controls -- joysticks :)
    Trigger elevatorManualTrigger = new Trigger(() -> Controls.applyDeadband(copilotJoystick.getLeftY()) != 0);
    elevatorManualTrigger.whileTrue(m_elevator.manualMoveCommand(() -> Controls.applyDeadband(-copilotJoystick.getLeftY())));
    
    Trigger armManualTrigger = new Trigger(() -> Controls.applyDeadband(copilotJoystick.getRightX()) != 0);
    armManualTrigger.whileTrue(m_arm.manualMoveCommand(() -> Controls.applyDeadband(copilotJoystick.getRightX())));
    

    m_collar.setDefaultCommand(m_collar.runCollarOff());
    m_climber.setDefaultCommand(m_climber.runClimberOff());

    m_elevator.setDefaultCommand(m_elevator.keepInPlacePIDCommand());
    m_arm.setDefaultCommand(m_arm.keepInPlacePIDCommand());
    m_ledSubsystem.setDefaultCommand(m_ledSubsystem.runGradientBlueYellow());


    // LED and rumble feedback when coral is seen in ramp
    m_rampSensor.coralSeenInRampTrigger().onTrue(
      new ParallelCommandGroup(
        // controller rumble
        new StartEndCommand(
          () -> {copilotJoystick.setRumble(RumbleType.kBothRumble, 1); driverJoystick.setRumble(RumbleType.kBothRumble, 1);},
          () -> {copilotJoystick.setRumble(RumbleType.kBothRumble, 0); driverJoystick.setRumble(RumbleType.kBothRumble, 0);})
          .withTimeout(1), 

        // set LED color
        m_ledSubsystem.runSolidGreen().withTimeout(1)));
      

    if (!RobotConstants.COMPETITION_MODE) { // don't use testJoystick in competition mode
      testJoystick.povCenter().negate().onTrue(new InstantCommand(() -> resetControllerConstantsSmartDashboard()));
      
      // run command runSolidGreen continuously if robot isWithinTarget()
      // m_vision.isWithinTargetTrigger(() -> drivetrain.getState().Pose).whileTrue(m_ledSubsystem.runSolidGreen());

      testJoystick.leftTrigger().and(testJoystick.a()).onTrue(m_lifter.moveToCommand(RobotState.L1_CORAL));
      testJoystick.leftTrigger().and(testJoystick.x()).onTrue(m_lifter.moveToCommand(RobotState.L2_CORAL));
      testJoystick.leftTrigger().and(testJoystick.b()).onTrue(m_lifter.moveToCommand(RobotState.L3_CORAL));
      testJoystick.leftTrigger().and(testJoystick.y()).onTrue(m_lifter.moveToCommand(RobotState.L4_CORAL));
      testJoystick.rightTrigger().and(testJoystick.x()).onTrue(m_lifter.moveToCommand(RobotState.INTAKE));
      testJoystick.rightTrigger().and(testJoystick.a()).whileTrue(m_collarCommandFactory.runCollarOut());
      testJoystick.rightTrigger().and(testJoystick.y()).whileTrue(m_collarCommandFactory.runCollarBackwards());
      

      // Run SysId routines when holding back/start and X/Y.
      // Note that each routine should be run exactly once in a single log.
      testJoystick.leftBumper().and(testJoystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
      testJoystick.leftBumper().and(testJoystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
      testJoystick.rightBumper().and(testJoystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
      testJoystick.rightBumper().and(testJoystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    } 

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

  private Command createScoreCommand(Command lifterCommand){
    return Commands.sequence(lifterCommand.asProxy(), m_collarCommandFactory.runCollarOut().withTimeout(0.5), m_collar.runCollarOffInstant(), m_lifter.moveToCommand(RobotState.INTAKE).asProxy());
  }

  public RobotContainer() {
    DataLogManager.start(); // log networktable 
    DriverStation.startDataLog(DataLogManager.getLog()); // log ds state, joystick data to /u/logs w/ usb stick, or home/lvuser/logs without. 
    DriverStation.silenceJoystickConnectionWarning(true); // turn off unplugged joystick errors 

    SignalLogger.enableAutoLogging(false);
    SignalLogger.start();
      
    if (!RobotConstants.COMPETITION_MODE) { // only initialize testJoystick if we're not in competition mode
      testJoystick = new CommandXboxController(OperatorConstants.kTestControllerPort);
    }

    // elevator moveTo auto commands
    NamedCommands.registerCommand("intakeCoral", Commands.sequence(m_collarCommandFactory.intakeCoralSequence().withTimeout(2), m_collar.runCollarOffInstant())); // update to use grapplehook instead
    NamedCommands.registerCommand("scoreL1Coral", createScoreCommand(m_lifter.moveToCommand(RobotState.L1_CORAL)));
    NamedCommands.registerCommand("scoreL2Coral", createScoreCommand(m_lifter.moveToCommand(RobotState.L2_CORAL)));
    NamedCommands.registerCommand("scoreL3Coral", createScoreCommand(m_lifter.moveToCommand(RobotState.L3_CORAL)));
    NamedCommands.registerCommand("scoreL4Coral", createScoreCommand(m_lifter.moveToCommand(RobotState.L4_CORAL)));


    // Build an auto chooser with all the PathPlanner autos. Uses Commands.none() as the default option
    
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

    SmartDashboard.putString("buildVersion/commitSHA", BuildConstants.GIT_SHA);
    SmartDashboard.putString("buildVersion/branchName", BuildConstants.GIT_BRANCH);
    SmartDashboard.putString("buildVersion/commitDate", BuildConstants.GIT_DATE);

    CameraServer.startAutomaticCapture();

    configureBindings();

  }

  // set motors to appropriate neutral modes for an enabled robot
  public void setEnabledNeutralMode() {
    drivetrain.setBrake(true);
    m_lifter.setEnabledNeutralMode();
    m_climber.setEnabledNeutralMode();

  }

  // set motors to appropriate neutral modes for an disabled robot
  public void setDisabledNeutralMode() {
    drivetrain.setBrake(false);
    m_lifter.setDisabledNeutralMode();
    m_climber.setDisabledNeutralMode();

  }

  public void setVoltageUponDisable() {
    CommandScheduler.getInstance().schedule(m_elevator.setVoltageToZeroCommand());
    CommandScheduler.getInstance().schedule(m_arm.setVoltageToZeroCommand());
    CommandScheduler.getInstance().schedule(m_collar.setVoltageToZeroCommand());
    CommandScheduler.getInstance().schedule(drivetrain.getCommandFromRequest(() -> drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0)));
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

  public void turnOffLifterKeepInPlace() {
    m_elevator.setRunKeepInPlace(false);
    m_arm.setRunKeepInPlace(false);

  }

  public void updatePoseEst() {

    EstimatedRobotPose estimatedRobotPose = m_vision.getEstimatedRobotPose();
    if (estimatedRobotPose != null && m_vision.getTargetFound()) {
      Pose3d estPose3d = estimatedRobotPose.estimatedPose; // estimated robot pose of vision
      Pose2d estPose2d = estPose3d.toPose2d();

        // check if new estimated pose and previous pose are less than 2 meters apart (fused poseEst)
        double distance = estPose2d.getTranslation().getDistance(drivetrain.getState().Pose.getTranslation());

        SmartDashboard.putNumber("vision/distanceBetweenVisionAndActualPose", distance);
        if (distance < VisionConstants.MAX_VISION_POSE_DISTANCE || !m_vision.isEstPoseJumpy()) {
          drivetrain.setVisionMeasurementStdDevs(m_vision.getCurrentStdDevs());
          drivetrain.addVisionMeasurement(estPose2d, Utils.fpgaToCurrentTime(estimatedRobotPose.timestampSeconds));
        
          m_estPoseField.setRobotPose(estPose2d);
          SmartDashboard.putNumberArray("vision/visionPose2dFiltered", new double[] {estPose2d.getX(), estPose2d.getY(), estPose2d.getRotation().getRadians()});
      }

      SmartDashboard.putNumberArray("vision/visionPose3D", new double[] {
        estPose3d.getX(),
        estPose3d.getY(),
        estPose3d.getZ(),
        estPose3d.getRotation().toRotation2d().getRadians()
      }); // post vision 3d to smartdashboard
    }
    

    m_actualField.setRobotPose(drivetrain.getState().Pose);
  }

}
