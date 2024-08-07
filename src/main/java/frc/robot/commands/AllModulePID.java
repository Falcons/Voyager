// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AllModulePID extends ParallelCommandGroup {
  private final SwerveSubsystem swerveSubsystem;
  public AllModulePID(SwerveSubsystem swerve) {
    this.swerveSubsystem = swerve;
    addCommands(
    swerve.modulePIDTuning("Front Left"),
    swerve.modulePIDTuning("Front Right"),
    swerve.modulePIDTuning("Back Left"),
    swerve.modulePIDTuning("Back Right"),
    new InstantCommand(swerve::allModuleSetpoint)
    );
  }
}