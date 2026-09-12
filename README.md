# Robot-Code-2025

Robot code for FRC Team 6418 (The Missfits), 2025 season (Reefscape). Java, WPILib
command-based, CTRE Phoenix 6 swerve, PhotonVision for AprilTags.

The goal of 2025's game is to pick up "coral" (PVC pipes) and place it on a reef at four heights, plus removing algae (big exercise balls). 

## Hardware

- CTRE swerve: TalonFX drive and steer, CANcoders, Pigeon 2 gyro.
- Elevator and arm ("lifter"), a collar that intakes and scores coral, and a climber. All TalonFX.
- Two Arducam OV9281 cameras with a Orange Pi 5 running PhotonVision (multi-tag PnP). 
- Grapple LaserCAN on the ramp to detect coral.
- Two robots: Dynamene (competition) and Ceridwen (practice), each with its own constants.

## Dependencies

- WPILib / GradleRIO 2025.3.1
- CTRE Phoenix 6 25.3.1
- PhotonLib v2025.2.1
- PathPlannerLib 2025.2.5
- libgrapplefrc 2025.1.3 (LaserCAN)

## Elevator and arm

Each mechanism runs a trapezoid profile and computes its own feedforward
(`ElevatorFeedforward` / `ArmFeedforward` with kS, kG, kV) on top of position PID, rather
than using Motion Magic. `ElevatorSubsystem` and `ArmSubsystem` each expose
`moveToCommand`.

Target positions are an enum, each holding an elevator height and arm angle. 
`LifterCommandFactory.moveToCommand(RobotState)` handles the sequencing: 
the arm inside the robot before the elevator moves, and the elevator 
has to be above a minimum height before the arm can swing out. 
This is gated by `isArmInsideRobotTrigger`. 
Level 4 has its own path because the arm goes over the top, and is gated 
by the `okToMoveArmBackTrigger`, since it has to avoid the coral input ramp. 

## Automatic alignment

`DriveToReefCommand` picks the closest reef AprilTag to the current pose, offsets the target
by robot width and left/right branch offset, and drives there with profiled PID on x and y
while holding the heading parallel to the tag. There's an intermediate waypoint in front of
the target so the robot doesn't approach at an angle. See `images/` for the visualized math.

## Localization

Two cameras, each wrapped in `LocalizationCamera`. Std devs scale with number of tags and
average tag distance; single-tag readings are trusted less. `VisionSubsystem` takes the
best reading each loop and feeds it to the swerve pose estimator. 

## Autos

We use PathPlanner. Paths and autos are in `src/main/deploy/pathplanner/`; named commands are
registered in `RobotContainer`. We ran 1-piece center and 3-piece left/right autos.

## Project Structure

```
src/main/java/frc/robot/
├── Robot.java
├── RobotContainer.java # Init, bindings, auto chooser, robot selection
├── RobotState.java # Elevator/arm setpoints per scoring position
├── Constants.java
├── VisionUtils.java # Closest-reef-tag lookup
├── commands/
│ ├── Autos.java
│ ├── DriveToReefCommand.java # Auto-align
│ └── PIDToTargetCommand.java
├── subsystems/
│ ├── lifter/ # Elevator, Arm, LifterCommandFactory
│ ├── collar/ # Collar, RampSensorSubsystem, CollarCommandFactory
│ ├── climber/
│ ├── CommandSwerveDrivetrain.java
│ ├── LocalizationCamera.java
│ ├── VisionSubsystem.java
│ └── LEDSubsystem.java
└── generated/ # TunerConstants for each robot

src/main/deploy/pathplanner/ # Paths and autos
```


## Running it

Standard WPILib project: open in VS Code with the WPILib extension, deploy to the roboRIO.