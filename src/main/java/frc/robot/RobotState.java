package frc.robot;

import frc.robot.Constants.RobotStateConstants;

public enum RobotState { 
    L1_CORAL(RobotStateConstants.C1_ELEVATOR_POS, RobotStateConstants.C1_ARM_POS), 
    L2_CORAL(RobotStateConstants.C2_ELEVATOR_POS, RobotStateConstants.C2_ARM_POS), 
    L2_CORAL_WITH_CORAL(RobotStateConstants.C2_ELEVATOR_POS_WITH_CORAL, RobotStateConstants.C2_ARM_POS_WITH_CORAL), 
    L3_CORAL(RobotStateConstants.C3_ELEVATOR_POS, RobotStateConstants.C3_ARM_POS), 
    L3_CORAL_WITH_CORAL(RobotStateConstants.C3_ELEVATOR_POS_WITH_CORAL, RobotStateConstants.C3_ARM_POS_WITH_CORAL), 
    L4_CORAL(RobotStateConstants.C4_ELEVATOR_POS, RobotStateConstants.C4_ARM_POS), 
    L4_CORAL_WITH_CORAL(RobotStateConstants.C4_ELEVATOR_POS_WITH_CORAL, RobotStateConstants.C4_ARM_POS_WITH_CORAL), 
    L2_ALGAE(RobotStateConstants.A2_ELEVATOR_POS, RobotStateConstants.A2_ARM_POS), 
    L3_ALGAE(RobotStateConstants.A3_ELEVATOR_POS, RobotStateConstants.A3_ARM_POS), 
    INTAKE(RobotStateConstants.IN_ELEVATOR_POS, RobotStateConstants.IN_ARM_POS),
    CLIMB(RobotStateConstants.CLIMB_ELEVATOR_POS, RobotStateConstants.CLIMB_ARM_POS);

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