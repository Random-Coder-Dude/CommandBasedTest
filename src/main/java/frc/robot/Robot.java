// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.CommandRunner;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    Logger.recordMetadata("Project Name", BuildInfo.MAVEN_NAME);
    Logger.recordMetadata("Build Date", BuildInfo.BUILD_DATE);
    Logger.recordMetadata("Git Hash", BuildInfo.GIT_SHA);
    Logger.recordMetadata("Git Date", BuildInfo.GIT_DATE);
    Logger.recordMetadata("Git Branch", BuildInfo.GIT_BRANCH);
    Logger.recordMetadata("Robot Name", NetworkUtils.getIdentity().name());
    Logger.recordMetadata("Robot MAC", NetworkUtils.getMACAddress());

    switch (BuildInfo.DIRTY) {
      case 0:
        Logger.recordMetadata("Git Dirty", "All Changes Commited");
        break;
      case 1:
        Logger.recordMetadata("Git Dirty", "Uncommited Changes");
        break;
      default:
        Logger.recordMetadata("Git Dirty", "Unknown");
        break;
    }

    SignalLogger.enableAutoLogging(false);

    if (Robot.isReal()) {
      Logger.addDataReceiver(new WPILOGWriter(Constants.LOG_PATH));
    }

    if (!DriverStation.isFMSAttached()) {
      Logger.addDataReceiver(new NT4Publisher());
    }

    Logger.start();

    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    if (Robot.isSimulation()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }

    CommandRunner.run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
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
}
