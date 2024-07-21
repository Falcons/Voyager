// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ModuleConstants;
import frc.robot.commands.DisplayValues;
import frc.robot.commands.PIDTuning;
import frc.robot.subsystems.SwerveModule;

public class RobotContainer {
  private final CommandXboxController driver = new CommandXboxController(0);

  private final SwerveModule frontLeft = new SwerveModule(
    ModuleConstants.frontLeftDriveCANID, 
    ModuleConstants.frontLeftTurningCANID, 
    true, 
    ModuleConstants.frontLeftAbsoluteOffset);

  private final SwerveModule frontRight = new SwerveModule(
    ModuleConstants.frontRightDriveCANID, 
    ModuleConstants.frontRightTurningCANID, 
    true, 
    ModuleConstants.frontRightAbsoluteOffset);
  
  private final SwerveModule backLeft = new SwerveModule(
    ModuleConstants.backLeftDriveCANID, 
    ModuleConstants.backLeftTurningCANID, 
    true, 
    ModuleConstants.backLeftAbsoluteOffset);

  private final SwerveModule backRight = new SwerveModule(
    ModuleConstants.backRightDriveCANID, 
    ModuleConstants.backRightTurningCANID, 
    true, 
    ModuleConstants.backRightAbsoluteOffset);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    driver.a().whileTrue(new PIDTuning(frontRight, 0));
    driver.start().onFalse(new DisplayValues(frontLeft, frontRight, backLeft, backRight).ignoringDisable(true));    
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
