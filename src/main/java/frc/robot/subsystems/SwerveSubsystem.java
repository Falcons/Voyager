// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveSubsystem extends SubsystemBase {
  private final SwerveModule frontLeft = new SwerveModule(
    "Front Left",
    ModuleConstants.frontLeftDriveCANID, 
    ModuleConstants.frontLeftTurningCANID, 
    true, 
    ModuleConstants.frontLeftAbsoluteOffset);

  private final SwerveModule frontRight = new SwerveModule(
    "Front Right",
    ModuleConstants.frontRightDriveCANID, 
    ModuleConstants.frontRightTurningCANID, 
    true, 
    ModuleConstants.frontRightAbsoluteOffset);
  
  private final SwerveModule backLeft = new SwerveModule(
    "Back Left",
    ModuleConstants.backLeftDriveCANID, 
    ModuleConstants.backLeftTurningCANID, 
    true, 
    ModuleConstants.backLeftAbsoluteOffset);

  private final SwerveModule backRight = new SwerveModule(
    "Back Right",
    ModuleConstants.backRightDriveCANID, 
    ModuleConstants.backRightTurningCANID, 
    true, 
    ModuleConstants.backRightAbsoluteOffset);

  SwerveModule[] modules = new SwerveModule[] {
    frontLeft,
    frontRight,
    backLeft,
    backRight
  };

  public Map<String, SwerveModule> swerveMap = Map.of(
    "Front Left", frontLeft, 
    "Front Right", frontRight,
    "Back Left", backLeft,
    "Back Right", backRight);
  
  
  private final Pigeon2 gyro = new Pigeon2(DriveConstants.pigeonCANID);

  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
    DriveConstants.kDriveKinematics, 
    getRotation2d(), 
    getModulePositions(), 
    new Pose2d());

  private final Field2d field = new Field2d();

  StructArrayPublisher<SwerveModuleState> statePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("SwerveStates/Actual", SwerveModuleState.struct).publish();
  StructPublisher<Pose2d> posPublisher = NetworkTableInstance.getDefault().getStructTopic("SwervePose/Actual", Pose2d.struct).publish();

  public SwerveSubsystem() {
    //copied from 0toAuto swerve video, waits 1 second for gyro cal. to zero heading
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        zeroHeading();
      } catch (Exception e) {
      }
    }).start();

    resetPose(new Pose2d());
    
  }

  @Override
  public void periodic() {
    statePublisher.set(getModuleState());
    posPublisher.set(odometry.getPoseMeters());

    updateOdometry();

    field.setRobotPose(odometry.getPoseMeters());

    SmartDashboard.putData(field);
    SmartDashboard.putNumber("Robot Heading", getHeading());
    SmartDashboard.putNumber("Robot Heading Radians", getRotation2d().getRadians());

    /*
    SmartDashboard.putNumber("Front Left Angle", frontLeft.getState().angle.getDegrees());
    SmartDashboard.putNumber("Front Right Angle", frontRight.getState().angle.getDegrees());
    SmartDashboard.putNumber("Back Left Angle", backLeft.getState().angle.getDegrees());
    SmartDashboard.putNumber("Back Right Angle", backRight.getState().angle.getDegrees());
    */
    for (SwerveModule module:modules) {
      SmartDashboard.putNumber(module.moduleName + " Angle", module.getState().angle.getDegrees());
    }

  }

  public void zeroHeading() {
    gyro.reset();
  }
  //test on timed first
  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360);
  }

  public Rotation2d getRotation2d() {
    return gyro.getRotation2d();
  }

  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, ModuleConstants.driveFreeSpeedMPS);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  public SwerveModuleState[] getModuleState() {
    return new SwerveModuleState[] {
      frontLeft.getState(),
      frontRight.getState(),
      backLeft.getState(),
      backRight.getState()
    };
  }

  public void updateOdometry() {
    odometry.update(getRotation2d(), getModulePositions());
  }

  public void resetPose(Pose2d pose) {
    odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()
    };
  }

  public RunCommand pidTuningFL() {
    return new RunCommand(() -> frontLeft.pidTuning(), this);
  }

  public RunCommand pidTuningFR() {
    return new RunCommand(() -> frontRight.pidTuning(), this);
  }

  public RunCommand pidTuningBL() {
    return new RunCommand(() -> backLeft.pidTuning(), this);
  }

  public RunCommand pidTuningBR() {
    return new RunCommand(() -> backRight.pidTuning(), this);
  }

  public RunCommand pidTuningAll(String str) {
    SwerveModule mod = swerveMap.get(str);
    return new RunCommand(() -> mod.pidTuning(), this);
  }

  public FunctionalCommand pidTuningFunc(String str) {
    SwerveModule mod = swerveMap.get(str);
    return new FunctionalCommand(
      mod::pidReset,
      mod::pidTuning,
      interrupted -> mod.stop(),
      () -> false,
      this);
  }

}
