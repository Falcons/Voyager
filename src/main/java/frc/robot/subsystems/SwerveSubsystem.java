// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveSubsystem extends SubsystemBase {
  private final SwerveModule frontLeft = new SwerveModule(
    "Front Left",
    ModuleConstants.frontLeftDriveCANID, 
    ModuleConstants.frontLeftTurningCANID, 
    ModuleConstants.frontLeftReversed, 
    ModuleConstants.frontLeftAbsoluteOffset);

  private final SwerveModule frontRight = new SwerveModule(
    "Front Right",
    ModuleConstants.frontRightDriveCANID, 
    ModuleConstants.frontRightTurningCANID, 
    ModuleConstants.frontRightReversed, 
    ModuleConstants.frontRightAbsoluteOffset);
  
  private final SwerveModule backLeft = new SwerveModule(
    "Back Left",
    ModuleConstants.backLeftDriveCANID, 
    ModuleConstants.backLeftTurningCANID, 
    ModuleConstants.backLeftReversed, 
    ModuleConstants.backLeftAbsoluteOffset);

  private final SwerveModule backRight = new SwerveModule(
    "Back Right",
    ModuleConstants.backRightDriveCANID, 
    ModuleConstants.backRightTurningCANID, 
    ModuleConstants.backRightReversed, 
    ModuleConstants.backRightAbsoluteOffset);

  private final Pigeon2 gyro = new Pigeon2(DriveConstants.pigeonCANID);

  private final PIDController xPID = new PIDController(2.3, 0, 0);
  private final PIDController yPID = new PIDController(2.3, 0, 0);
  private final PIDController rotationPID = new PIDController(2.5, 1.2, 0);

  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
    DriveConstants.kDriveKinematics, 
    getRotation2d(), 
    getModulePositions(), 
    new Pose2d());

  private final Field2d field = new Field2d();

  private final SwerveModule[] modules = new SwerveModule[] {
    frontLeft,
    frontRight,
    backLeft,
    backRight
  };

  private final Map<String, SwerveModule> swerveMap = Map.of(
    "Front Left", frontLeft, 
    "Front Right", frontRight,
    "Back Left", backLeft,
    "Back Right", backRight);

  private final Map<Character, PIDController> pidMap = Map.of(
    'x', xPID,
    'y', yPID,
    'o', rotationPID);

  StructArrayPublisher<SwerveModuleState> commandedStatePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("SwerveStates/Commanded", SwerveModuleState.struct).publish();
  StructArrayPublisher<SwerveModuleState> statePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("SwerveStates/Actual", SwerveModuleState.struct).publish();
  StructPublisher<Pose2d> posPublisher = NetworkTableInstance.getDefault().getStructTopic("SwervePose/Actual", Pose2d.struct).publish();

  public SwerveSubsystem() {
    System.out.println("Drive Max" + ModuleConstants.driveMaxSpeedMPS);
    System.out.println("Angle Max" + DriveConstants.maxAngularSpeedRadiansPerSecond);
    rotationPID.enableContinuousInput(-Math.PI, Math.PI);
    rotationPID.setIZone(0.05);

    xPID.reset();
    yPID.reset();
    rotationPID.reset();

    //copied from 0toAuto swerve video, waits 1 second for gyro calibration to zero heading
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        zeroHeading();
      } catch (Exception e) {
      }
    }).start();

    resetPose(new Pose2d());

    SmartDashboard.putData("X PID", xPID);
    SmartDashboard.putData("Y PID", yPID);
    SmartDashboard.putData("Rotation PID", rotationPID);
    SmartDashboard.putNumber("Module Setpoint", 0);
  
    // PathPlanner init
    AutoBuilder.configureHolonomic(
      this::getPose, 
      this::resetPose, 
      this::getChassisSpeeds, 
      this::driveRobotRelative, 
      new HolonomicPathFollowerConfig(
        new PIDConstants(2.3, 0, 0),
        new PIDConstants(2.5, 1.2, 0),
        ModuleConstants.driveMaxSpeedMPS * 0.15, //15% of max speed
        DriveConstants.driveBaseRadius, 
        new ReplanningConfig()
      ),
      //Returns true to flip path if on red alliance
      () -> {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      }, 
      this);
  }

  @Override
  public void periodic() {
    //Sends actual Module States and Pose over NT
    statePublisher.set(getModuleStates());
    posPublisher.set(odometry.getPoseMeters());

    updateOdometry();

    field.setRobotPose(odometry.getPoseMeters());

    SmartDashboard.putData(field);

    SmartDashboard.putNumber("FieldX", getPoseX());
    SmartDashboard.putNumber("FieldY", getPoseY());
    SmartDashboard.putNumber("Robot Heading Degrees", getRotation2d().getDegrees());
    SmartDashboard.putNumber("Robot Heading Radians", getRotation2d().getRadians());
    SmartDashboard.putNumber("Robot Heading Method", getHeadingRadians());

    for (SwerveModule module:modules) {
      SmartDashboard.putNumber("Module/Speed/" + module.moduleName, module.getState().speedMetersPerSecond);
      SmartDashboard.putNumber("Module/Angle/" + module.moduleName, module.getState().angle.getDegrees());
    }
    SmartDashboard.putNumber("Rotation PID setpoint", robotPIDSetpoint('o'));
    SmartDashboard.putNumber("Robot/X Speed", getChassisSpeeds().vxMetersPerSecond);
    SmartDashboard.putNumber("Robot/Y Speed", getChassisSpeeds().vyMetersPerSecond);
    SmartDashboard.putNumber("Robot/Turning Speed", getChassisSpeeds().omegaRadiansPerSecond);
  }

  /** @return Gyro Position in degrees (-180 to 180) CCW+ */
  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360);
  }

  public double getHeadingRadians() {
    return Math.IEEEremainder(getRotation2d().getRadians(), 2 * Math.PI);
  }

  public void zeroHeading() {
    gyro.reset();
  }

  public Rotation2d getRotation2d() {
    return gyro.getRotation2d();
  }

  /** Stops all Swerve Motors */
  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  /** Sets all 4 Modules to specified Speed and Angle */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    commandedStatePublisher.set(desiredStates);
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, ModuleConstants.driveMaxSpeedMPS);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      frontLeft.getState(),
      frontRight.getState(),
      backLeft.getState(),
      backRight.getState()
    };
  }

  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(moduleStates);
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()
    };
  }

  public void resetDriveEncoders() {
    frontLeft.resetEncoders();
    frontRight.resetEncoders();
    backLeft.resetEncoders();
    backRight.resetEncoders();
  }

  public void updateOdometry() {
    odometry.update(getRotation2d(), getModulePositions());
  }

  public void resetPose(Pose2d pose) {
    odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  public double getPoseX() {
    return odometry.getPoseMeters().getX();
  }

  public double getPoseY() {
    return odometry.getPoseMeters().getY();
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Method for any Robot PID calculation
   * @param controller 'x', 'y', or 'o' for xPID, yPID, rotationPID
   * @param measurement sensor to read
   * @param setpoint target value
   * @return output of chosen PIDController
   */
  public double robotPIDCalc(char controller, double measurement, double setpoint) {
    PIDController pid = pidMap.get(controller);
    return pid.calculate(measurement, setpoint);
  }

  public double robotPIDSetpoint(char controller) {
    PIDController pid = pidMap.get(controller);
    return pid.getSetpoint();
  }

  /**
   * Commands Swerve Module to Setpoint
   * @param moduleName Name of Swerve Module (e.g. "Front Left")
   */
  public FunctionalCommand modulePIDTuning(String moduleName) {
    SwerveModule mod = swerveMap.get(moduleName);
    return new FunctionalCommand(
      mod::pidReset,
      mod::pidTuning,
      interrupted -> mod.stop(),
      () -> false);
  }

  public void allModuleSetpoint() {
    frontLeft.setpoint();
    frontRight.setpoint();
    backLeft.setpoint();
    backRight.setpoint();
  }
}
