// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.RobotConstants;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private Timer m_neutralModeTimer = new Timer();

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
    
    m_robotContainer.updatePoseEst();
  }

  @Override
  public void disabledInit() {
    m_neutralModeTimer.start();
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.turnOffLifterKeepInPlace();
    m_robotContainer.setVoltageUponDisable();
  }

  @Override
  public void disabledPeriodic() {
    // if DISABLED_COAST_DELAY seconds have passed since disabling, set the neutral mode to coast
    if (m_neutralModeTimer.get() > RobotConstants.DISABLED_COAST_DELAY) {
      m_robotContainer.setDisabledNeutralMode();
      m_neutralModeTimer.stop();
      m_neutralModeTimer.reset();
    }
  }

  @Override
  public void disabledExit() {
    m_robotContainer.setEnabledNeutralMode();
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
  
}
