// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Conversion;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Misc;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystick extends Command {
  
  private final SwerveSubsystem swerveSubsystem;
  private final Supplier<Double> xSpeedFunc, ySpeedFunc, turningSpeedFunc;
  private final Supplier<Boolean> fieldOriented;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
  public SwerveJoystick(
    SwerveSubsystem swerveSubsystem, 
    Supplier<Double> xSpd, 
    Supplier<Double> ySpd,
    Supplier<Double> turnSpd,
    Supplier<Boolean> field) {
    this.swerveSubsystem = swerveSubsystem;
    this.xSpeedFunc = xSpd;
    this.ySpeedFunc = ySpd;
    this.turningSpeedFunc = turnSpd;
    this.fieldOriented = field;
    //rate limit value from 0ToAuto
    this.xLimiter = new SlewRateLimiter(3);
    this.yLimiter = new SlewRateLimiter(3);
    this.turningLimiter = new SlewRateLimiter(3);
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("SwerveJoystick Start");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = xSpeedFunc.get();
    double ySpeed = ySpeedFunc.get();
    double turningSpeed = turningSpeedFunc.get();

    SmartDashboard.putNumber("X Speed", xSpeed);
    SmartDashboard.putNumber("Y Speed", ySpeed);
    SmartDashboard.putNumber("Turning Speed", turningSpeed);

    xSpeed = Math.abs(xSpeed) > Misc.kDeadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > Misc.kDeadband ? ySpeed : 0.0;
    turningSpeed = Math.abs(turningSpeed) > Misc.kDeadband ? turningSpeed : 0.0;

    xSpeed = xLimiter.calculate(xSpeed) * ModuleConstants.freeSpeedMpS / 4.0;
    ySpeed = yLimiter.calculate(ySpeed) * ModuleConstants.freeSpeedMpS / 4.0;
    turningSpeed = turningLimiter.calculate(turningSpeed) * ModuleConstants.maxDegreeperSecond * Conversion.degToRad / 4.0;

    ChassisSpeeds chassisSpeeds;
    if (fieldOriented.get()) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
    } else {
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    }

    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    SmartDashboard.putNumber("Back Right Set Speed", moduleStates[3].speedMetersPerSecond);
    SmartDashboard.putNumber("Back Right Set Angle", moduleStates[3].angle.getDegrees());
    
    //swerveSubsystem.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}