// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwervePositionPIDTuning extends Command {
  
  private final SwerveSubsystem swerveSubsystem;

  public SwervePositionPIDTuning(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("SwervePos Start");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = swerveSubsystem.robotPIDCalc('x', swerveSubsystem.getPose().getX(), 0);
    double ySpeed = swerveSubsystem.robotPIDCalc('y', swerveSubsystem.getPose().getY(), 0);
    double rotSpeed = swerveSubsystem.robotPIDCalc('o', swerveSubsystem.getWrappedHeadingRadians(), 0);

    SmartDashboard.putNumber("PID Output/X", xSpeed);
    SmartDashboard.putNumber("PID Output/Y", ySpeed);
    SmartDashboard.putNumber("PID Output/O", rotSpeed);

    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, swerveSubsystem.getRotation2d());

    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    swerveSubsystem.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
    System.out.println("SwervePos End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
