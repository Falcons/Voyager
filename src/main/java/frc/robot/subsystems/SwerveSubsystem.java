// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveSubsystem extends SubsystemBase {
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
  
  //FIXXXX
  private final Pigeon2 gyro = new Pigeon2(DriveConstants.pigeonCANID);

  public SwerveSubsystem() {
    //copied from 0toAuto swerve video, waits 1 second for gyro cal. to zero heading
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        zeroHeading();
      } catch (Exception e) {
      }
    }).start();
    
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Robot Heading", getHeading());

    SmartDashboard.putNumber("Front Left Speed", frontLeft.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Front Right Speed", frontRight.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Back Left Speed", backLeft.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Back Right Speed", backRight.getState().speedMetersPerSecond);

    SmartDashboard.putNumber("Front Left Angle", frontLeft.getState().angle.getDegrees());
    SmartDashboard.putNumber("Front Right Angle", frontRight.getState().angle.getDegrees());
    SmartDashboard.putNumber("Back Left Angle", backLeft.getState().angle.getDegrees());
    SmartDashboard.putNumber("Back Right Angle", backRight.getState().angle.getDegrees());
  }

  public void zeroHeading() {
    gyro.reset();
  }
  //test on timed first
  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360);
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, ModuleConstants.freeSpeedMpS);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }
}
