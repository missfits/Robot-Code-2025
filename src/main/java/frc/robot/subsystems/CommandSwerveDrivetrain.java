package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.struct.ChassisSpeedsStruct;
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
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.generated.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedOperatorPerspective = false;
    
    private Pigeon2 m_gyro = getPigeon2();

    private StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault().getStructArrayTopic("drivetrain/actualModuleStates", SwerveModuleState.struct).publish();
    private StructArrayPublisher<SwerveModuleState> targetPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("drivetrain/targetModuleStates", SwerveModuleState.struct).publish();
    private DoubleArrayPublisher voltagePublisher = NetworkTableInstance.getDefault().getDoubleArrayTopic("drivetrain/moduleVoltages").publish();
    private DoublePublisher rotationPublisher = NetworkTableInstance.getDefault().getDoubleTopic("drivetrain/rotation").publish();
    
    private StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault().getStructTopic("drivetrain/pose", Pose2d.struct).publish();

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

        AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(DrivetrainConstants.ROBOT_DRIVE_P, DrivetrainConstants.ROBOT_DRIVE_I, DrivetrainConstants.ROBOT_DRIVE_D), // Translation PID constants
                    new PIDConstants(DrivetrainConstants.ROBOT_STEER_P, DrivetrainConstants.ROBOT_STEER_I, DrivetrainConstants.ROBOT_STEER_D), // Rotation PID constants
                    4.5, // default - Max module speed, in m/s
                    0.43105229381, // guess? - Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig(false, false) // Default path replanning config. See the API for the options here
            ),
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


    // get robot's pose2d
    public Pose2d getPose() {
        return this.getState().Pose;
    }

    // reset drivetrain odometry to a certain pose (pose2d only)
    public void resetPose(Pose2d pose) {
        this.seedFieldRelative(pose);
    }

    // get robot's chassisspeeds
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return this.getState().speeds;
    }

    // drive robot from chassis speeds 
    public void driveRobotRelative(ChassisSpeeds speeds) {
        this.setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(speeds));

    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
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

    @Override
    public void periodic() {
        SwerveModuleState[] swerveModules = {this.getModule(0).getTargetState(), this.getModule(1).getTargetState(), this.getModule(2).getTargetState(), this.getModule(3).getTargetState()};
        double[] voltages = {this.getModule(0).getDriveMotor().getMotorVoltage().getValue(), this.getModule(1).getDriveMotor().getMotorVoltage().getValue(), this.getModule(2).getDriveMotor().getMotorVoltage().getValue(), this.getModule(3).getDriveMotor().getMotorVoltage().getValue()};
       
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

        publisher.set(this.m_moduleStates);

        targetPublisher.set(swerveModules);
        voltagePublisher.set(voltages);

        rotationPublisher.set(m_gyro.getAngle());
        posePublisher.set(getPose());


    }
}
