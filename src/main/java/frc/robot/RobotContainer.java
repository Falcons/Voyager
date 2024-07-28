// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.SwerveJoystick;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  private final CommandXboxController driver = new CommandXboxController(0);

  private final SwerveSubsystem swerve = new SwerveSubsystem();

  public RobotContainer() {
    /*
    swerve.setDefaultCommand(new SwerveJoystick(
      swerve, 
      () -> -driver.getLeftY(), 
      () -> -driver.getLeftX(), 
      () -> -driver.getRightX(), 
      () -> !driver.getHID().getLeftBumper()));
    */

    configureBindings();

    SmartDashboard.putData("Set Pose", new InstantCommand(() -> swerve.resetPose(DriveConstants.noteBlueCloseAmp)));
    SmartDashboard.putData("Swerve Subsystem", swerve);
  }

  private void configureBindings() {
    driver.b().onTrue(new InstantCommand(swerve::zeroHeading));

    driver.povUpLeft().whileTrue(swerve.pidTuningFunc("Front Left"));
    driver.povUpRight().whileTrue(swerve.pidTuningFunc("Front Right"));
    driver.povDownLeft().whileTrue(swerve.pidTuningFunc("Back Left"));
    driver.povDownRight().whileTrue(swerve.pidTuningFunc("Back Right"));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
