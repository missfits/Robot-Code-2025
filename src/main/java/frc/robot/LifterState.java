package frc.robot;

import frc.robot.Constants.LifterStateConstants;

public enum LifterState { 
    L1_CORAL(LifterStateConstants.C1_ELEVATOR_POS, LifterStateConstants.C1_ARM_POS), 
    L2_CORAL(LifterStateConstants.C2_ELEVATOR_POS, LifterStateConstants.C2_ARM_POS), 
    L3_CORAL(LifterStateConstants.C3_ELEVATOR_POS, LifterStateConstants.C3_ARM_POS), 
    L4_CORAL(LifterStateConstants.C4_ELEVATOR_POS, LifterStateConstants.C4_ARM_POS), 
    L2_ALGAE(LifterStateConstants.A2_ELEVATOR_POS, LifterStateConstants.A2_ARM_POS), 
    L3_ALGAE(LifterStateConstants.A3_ELEVATOR_POS, LifterStateConstants.A3_ARM_POS), 
    INTAKE(LifterStateConstants.IN_ELEVATOR_POS, LifterStateConstants.IN_ARM_POS);

    private final double elevatorPos;
    private final double armPos;

    // constructor
    private LifterState(double ep, double ap) {
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