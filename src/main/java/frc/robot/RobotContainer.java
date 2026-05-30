// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.CommandRegisterer;
import frc.robot.Swerve.Commands.SwerveCommand;
import frc.robot.Swerve.Commands.SwerveSysId;
import frc.robot.Swerve.Generated.TunerConstants;
import frc.robot.Swerve.Subsystem.SwerveSubsystem;

public class RobotContainer {
  private CommandXboxController driveController;
  private CommandXboxController opController;
  private final SwerveSubsystem swerveSubsystem;

  public RobotContainer() {
    driveController = new CommandXboxController(Constants.Driver.kDriverControllerPort);
    opController = new CommandXboxController(Constants.Operator.kOperatorControllerPort);
    swerveSubsystem = TunerConstants.createDrivetrain();
    configureBindings();
  }

  private void configureBindings() {
    CommandRegisterer.register(
        new SwerveCommand(
            swerveSubsystem,
            () -> driveController.getLeftX(),
            () -> -driveController.getLeftY(),
            () -> driveController.getRightX(),
            () -> false,
            () -> false,
            () -> driveController.getHID().getAButton(),
            () -> false,
            () -> false,
            () -> false,
            () -> new Pose2d()));

    CommandRegisterer.register(
        new SwerveSysId(swerveSubsystem, () -> opController.getHID().getAButtonPressed()));

    swerveSubsystem.register();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
