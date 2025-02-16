package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.struct.ChassisSpeedsStruct;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.StructArrayLogEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.generated.TunerConstants;

import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;


/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedOperatorPerspective = false;

    /** Swerve request to apply during robot-centric path following */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    
    private Pigeon2 m_gyro = getPigeon2();

    private final Vector<N3> stateStdDevs = VecBuilder.fill(1, 1,1);
    private final Vector<N3> visionStdDevs = VecBuilder.fill(1, 1, 1);

    // fusePoseEstimator stores vision + drivetrain pose
    public final SwerveDrivePoseEstimator fusedPoseEstimator = new SwerveDrivePoseEstimator(
        this.getKinematics(), 
        this.getPigeon2().getRotation2d(), 
        this.getState().ModulePositions, 
        this.getState().Pose,
        stateStdDevs,
        visionStdDevs); 

        // drivePoseEstimator stores just drivetrain pose
        public final SwerveDrivePoseEstimator drivePoseEstimator = new SwerveDrivePoseEstimator(
            this.getKinematics(), 
            this.getPigeon2().getRotation2d(), 
            this.getState().ModulePositions, 
            this.getState().Pose); 
        
    private StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault().getStructArrayTopic("drivetrain/actualModuleStates", SwerveModuleState.struct).publish();
    private StructArrayPublisher<SwerveModuleState> targetPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("drivetrain/targetModuleStates", SwerveModuleState.struct).publish();
    private DoubleArrayPublisher voltagePublisher = NetworkTableInstance.getDefault().getDoubleArrayTopic("drivetrain/moduleVoltages").publish();
    private DoublePublisher rotationPublisher = NetworkTableInstance.getDefault().getDoubleTopic("drivetrain/rotation").publish();
    
    private StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault().getStructTopic("drivetrain/pose", Pose2d.struct).publish();

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    // units are rotor rotations, which should be correct with voltage based control. 
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    // units are wheel rotations, which should be correct with voltage based control. 
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(5), // Use dynamic voltage of 5 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );
        
    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineRotation;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }


        // 4.5, // default - Max module speed, in m/s
        // 0.43105229381 // guess? - Drive base radius in meters. Distance from robot center to furthest module.
        // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        RobotConfig config = null;
        try {
        config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
        // Handle exception as needed
        e.printStackTrace();
        }

        AutoBuilder.configure(
            () -> getState().Pose,   // Supplier of current robot pose
            this::setNewPose,         // Consumer for seeding pose against auto (will be called if your auto has a starting pose)
            () -> getState().Speeds, // Supplier of current robot speeds. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        // .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        // .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(DrivetrainConstants.ROBOT_POSITION_P, DrivetrainConstants.ROBOT_POSITION_I, DrivetrainConstants.ROBOT_POSITION_D), // Translation PID constants
                    new PIDConstants(DrivetrainConstants.ROBOT_ROTATION_P, DrivetrainConstants.ROBOT_ROTATION_I, DrivetrainConstants.ROBOT_ROTATION_D) // Rotation PID constants            
            ),
            config,
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
        );
    }

    public Command getCommandFromRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    public SwerveDrivePoseEstimator getFusedPoseEstimator(){
        return fusedPoseEstimator;
    }

    public SwerveDrivePoseEstimator getDrivePoseEstimator(){
        return drivePoseEstimator;
    }

    public Rotation2d getRobotRotation(){
        return this.getState().Pose.getRotation();
    } 
    
    public void updateRobotPoseWithFusedPoseEst(){
        this.resetPose(fusedPoseEstimator.update(
            this.getPigeon2().getRotation2d(), 
            this.getState().ModulePositions)); 
    }

    // doesn't change robot pose, hust stored SwerveDrivePoseEst
    public void updateDrivePoseWithOdometry(){
        drivePoseEstimator.update(
            this.getPigeon2().getRotation2d(),
            this.getState().ModulePositions);
    }

    // updates both fused pose and drivetrain only pose
    public void setNewPose(Pose2d newPose){
        drivePoseEstimator.resetPose(newPose);
        fusedPoseEstimator.resetPose(newPose);
        this.resetPose(newPose);
    }

    // resets fusedPoseEst to drivePoseEst
    public void resetRobotPoseWithDrivePoseEst(){
        setNewPose(drivePoseEstimator.getEstimatedPosition());
    }


    // sets all motors' (including steer) neutral modes to coast (false) or brake (true)
    public void setBrake(boolean brake) {
        this.configNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        this.configSteerNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    public StatusCode configSteerNeutralMode(NeutralModeValue neutralModeValue){
        StatusCode statusCode = StatusCode.OK;
        for (var module : getModules()){
            StatusCode result = module.getSteerMotor().setNeutralMode(neutralModeValue);
            if (!result.equals(StatusCode.OK)){
                statusCode = result;
            }
        }
        return statusCode;
    }

    @Override
    public void periodic() {
        SwerveModuleState[] currentStates = {this.getModule(0).getCurrentState(), this.getModule(1).getCurrentState(), this.getModule(2).getCurrentState(), this.getModule(3).getCurrentState()};
        SwerveModuleState[] targetStates = {this.getModule(0).getTargetState(), this.getModule(1).getTargetState(), this.getModule(2).getTargetState(), this.getModule(3).getTargetState()};
        double[] voltages = {this.getModule(0).getDriveMotor().getMotorVoltage().getValueAsDouble(), this.getModule(1).getDriveMotor().getMotorVoltage().getValueAsDouble(), this.getModule(2).getDriveMotor().getMotorVoltage().getValueAsDouble(), this.getModule(3).getDriveMotor().getMotorVoltage().getValueAsDouble()};
       
        /* Periodically try to apply the operator perspective */
        /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
        /* This allows us to correct the perspective in case the robot code restarts mid-match */
        /* Otherwise, only check and apply the operator perspective if the DS is disabled */
        /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                this.setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation
                                : BlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
            });
        }

        publisher.set(currentStates);

        targetPublisher.set(targetStates);
        voltagePublisher.set(voltages);

        rotationPublisher.set(m_gyro.getAngle());
        posePublisher.set(this.getState().Pose);


    }
}
