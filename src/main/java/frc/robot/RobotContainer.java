// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SwerveJoystick;
//import frc.robot.commands.AllModulePID;
//import frc.robot.commands.SwervePositionPIDTuning;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  private final CommandXboxController driver = new CommandXboxController(0);

  private final SwerveSubsystem swerve = new SwerveSubsystem();

  public RobotContainer() {
    swerve.setDefaultCommand(new SwerveJoystick(
      swerve, 
      () -> -driver.getLeftY(), 
      () -> -driver.getLeftX(), 
      () -> -driver.getRightX(), 
      () -> !driver.getHID().getLeftBumper()));

    configureBindings();

    SmartDashboard.putData("Reset Field Pose", new InstantCommand(() -> swerve.resetPose(new Pose2d())).ignoringDisable(true));
  }

  private void configureBindings() {
    driver.b().onTrue(new InstantCommand(swerve::zeroHeading));
/*
    driver.x().whileTrue(new SwervePositionPIDTuning(swerve));
    driver.a().whileTrue(new AllModulePID(swerve));

    driver.povUpLeft().whileTrue(swerve.modulePIDTuning("Front Left"));
    driver.povUpRight().whileTrue(swerve.modulePIDTuning("Front Right"));
    driver.povDownLeft().whileTrue(swerve.modulePIDTuning("Back Left"));
    driver.povDownRight().whileTrue(swerve.modulePIDTuning("Back Right"));
 */
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("better figure 8");
  }
}
