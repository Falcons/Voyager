// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

//import org.photonvision.PhotonCamera;
//import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

//import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

//import frc.robot.LimelightHelpers;

public class SwerveSubsystem extends SubsystemBase {
  private final SwerveModule frontLeft = new SwerveModule(
    "Front Left",
    ModuleConstants.frontLeftDriveCANID, 
    ModuleConstants.frontLeftTurningCANID,
    ModuleConstants.sparkMaxDataPort, //Use this constant (-1) if connected with Spark Max Breakout Board instead of RIO Analog IN
    ModuleConstants.frontLeftReversed, 
    ModuleConstants.frontLeftAbsoluteOffset);

  private final SwerveModule frontRight = new SwerveModule(
    "Front Right",
    ModuleConstants.frontRightDriveCANID, 
    ModuleConstants.frontRightTurningCANID,
    ModuleConstants.sparkMaxDataPort, 
    ModuleConstants.frontRightReversed, 
    ModuleConstants.frontRightAbsoluteOffset);
  
  private final SwerveModule backLeft = new SwerveModule(
    "Back Left",
    ModuleConstants.backLeftDriveCANID, 
    ModuleConstants.backLeftTurningCANID,
    ModuleConstants.sparkMaxDataPort, 
    ModuleConstants.backLeftReversed, 
    ModuleConstants.backLeftAbsoluteOffset);

  private final SwerveModule backRight = new SwerveModule(
    "Back Right",
    ModuleConstants.backRightDriveCANID, 
    ModuleConstants.backRightTurningCANID,
    0,
    ModuleConstants.backRightReversed, 
    ModuleConstants.backRightAbsoluteOffset);

  private final Pigeon2 gyro = new Pigeon2(DriveConstants.pigeonCANID);
  private double maxSpeed = ModuleConstants.driveMaxSpeedMPS;

  //PhotonCamera photonCam;

  private final PIDController xPID = new PIDController(DriveConstants.translationKP, 0, 0);
  private final PIDController yPID = new PIDController(DriveConstants.translationKP, 0, 0);
  private final PIDController rotationPID = new PIDController(DriveConstants.rotationKP, DriveConstants.rotationKI, 0);

  private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
    DriveConstants.kDriveKinematics, 
    getRotation2d(), 
    getModulePositions(), 
    new Pose2d());

  private final Field2d field2024 = new Field2d();

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

  private final DigitalInput opticalLimit = new DigitalInput(0);

  StructArrayPublisher<SwerveModuleState> commandedStatePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("SwerveStates/Commanded", SwerveModuleState.struct).publish();
  StructArrayPublisher<SwerveModuleState> statePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("SwerveStates/Actual", SwerveModuleState.struct).publish();
  //StructPublisher<Pose2d> posPublisher = NetworkTableInstance.getDefault().getStructTopic("SwervePose/Actual", Pose2d.struct).publish();

  public SwerveSubsystem() {
    //photonCam = new PhotonCamera("USB2.0_PC_CAMERA");
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

    // Sends PID Controllers to Shuffleboard
    SmartDashboard.putData("RobotPID/X PID", xPID);
    SmartDashboard.putData("RobotPID/Y PID", yPID);
    SmartDashboard.putData("RobotPID/Rotation PID", rotationPID);
    SmartDashboard.putNumber("Module Setpoint", 0);
  
    // PathPlanner Initialization
    AutoBuilder.configureHolonomic(
      this::getPose, 
      this::resetPose, 
      this::getChassisSpeeds, 
      this::driveRobotRelative, 
      new HolonomicPathFollowerConfig(
        new PIDConstants(2.3, 0, 0),
        new PIDConstants(2.5, 1.2, 0),
        ModuleConstants.driveMaxSpeedMPS,
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
    statePublisher.set(getModuleStates());
    //posPublisher.set(poseEstimator.getEstimatedPosition());

    updatePoseEstimator();
    field2024.setRobotPose(poseEstimator.getEstimatedPosition());
    
    for (SwerveModule module:modules) {
      SmartDashboard.putNumber("Module/Speed/" + module.moduleName, module.getState().speedMetersPerSecond);
      SmartDashboard.putNumber("Module/Angle/" + module.moduleName, module.getState().angle.getDegrees());
    }

    SmartDashboard.putNumber("Back Right Raw Abs", backRight.getAbsEncoderRaw());
    SmartDashboard.putNumber("Back Right Raw w offset", backRight.getRawPositionWithOffset());

    SmartDashboard.putNumber("Robot/FieldX", getPose().getX());
    SmartDashboard.putNumber("Robot/X Speed", getChassisSpeeds().vxMetersPerSecond);

    SmartDashboard.putNumber("Robot/FieldY", getPose().getY());
    SmartDashboard.putNumber("Robot/Y Speed", getChassisSpeeds().vyMetersPerSecond);

    SmartDashboard.putNumber("Robot/Heading Radians", getWrappedHeadingRadians());
    SmartDashboard.putNumber("Robot/Turning Speed", getChassisSpeeds().omegaRadiansPerSecond);

    SmartDashboard.putData(field2024);
    SmartDashboard.putBoolean("Limit/Get", opticalLimit.get());
    /*
    var result = photonCam.getLatestResult();
    boolean hasTargets = result.hasTargets();
    SmartDashboard.putBoolean("Has targets", hasTargets);

    if (hasTargets) {
      PhotonTrackedTarget target = result.getBestTarget();
      double area = target.getArea();
      SmartDashboard.putNumber("PhotonCam/Area", area);
    } 
    */
  }

  /** Stops all Swerve Motors */
  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
}

  public void slowMode(){
    this.maxSpeed = DriveConstants.slowModeSpeed;
  }
  public void fastMode(){
    this.maxSpeed = ModuleConstants.driveMaxSpeedMPS;
  }
  public void toggleMode(){
    if(this.maxSpeed == DriveConstants.slowModeSpeed) fastMode();
    else slowMode();
  }

// Gyro

  /** Resets Gyro Heading to 0 */
  public void zeroHeading() {
    gyro.reset();
  }

  /** @return Gyro Rotation2d, continuous */
  public Rotation2d getRotation2d() {
    return gyro.getRotation2d();
  }

  /** @return Gyro Heading in Radians (-Pi, Pi) CCW+ */
  public double getWrappedHeadingRadians() {
    return Math.IEEEremainder(getRotation2d().getRadians(), 2 * Math.PI);
  }

  /** @return Gyro Heading in Degrees(-180, 180) CCW+ */
  public double getWrappedHeadingDegrees() {
    return getWrappedHeadingRadians() * 180 / Math.PI;
  }

// SwerveModuleStates

  /** Sets all 4 Modules to specified Speed and Angle */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    commandedStatePublisher.set(desiredStates);
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxSpeed);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  /** @return Array of all 4 Module Speed and Angle */
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      frontLeft.getState(),
      frontRight.getState(),
      backLeft.getState(),
      backRight.getState()
    };
  }

// ChassisSpeeds

  /** @return Robot-Relative chassisSpeeds */
  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  /** Sets Module states from a chassisSpeeds */
  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(moduleStates);
  }

// Odometry/Pose Estimation

  /** Sets Robot Pose */
  public void resetPose(Pose2d pose) {
    //odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
    poseEstimator.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  /** @return Robot Pose */
  public Pose2d getPose() {
    //return odometry.getPoseMeters();
    return poseEstimator.getEstimatedPosition();
  }

  /** @return Array of current Position and Angle of all 4 Modules */
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()
    };
  }

  /** Updates Robot Pose based on Gyro and Module Positions */
  public void updatePoseEstimator() {
    poseEstimator.update(getRotation2d(), getModulePositions());
    /*
    boolean useMegaTag2 = false; 
    boolean doRejectUpdate = false;

    //using megatag 1
    if (!useMegaTag2) { 
      LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

      if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
        if (mt1.rawFiducials[0].ambiguity > 0.7) {
          doRejectUpdate = true;
        }
        if (mt1.rawFiducials[0].distToCamera > 3) {
          doRejectUpdate = true;
        }
      }

      if (mt1.tagCount == 0) {
        doRejectUpdate = true;
      }

      if(!doRejectUpdate) {
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, 9999999)); //StdDev from Limelight website
        poseEstimator.addVisionMeasurement(mt1.pose, mt1.timestampSeconds);
      }
      
    //using megatag 2 (updated)
    } else {
      LimelightHelpers.SetRobotOrientation("limelight", poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

      if (Math.abs(gyro.getRate()) > 720) {
        doRejectUpdate = true;
      }
      if (mt2.tagCount == 0) {
        doRejectUpdate = true;
      }

      if (!doRejectUpdate) {
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999)); //StdDev from Limelight website
        poseEstimator.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
      }
    }
     */
  }

// PID

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

  /** Sets all 4 Module Setpoint from Smartdashboard value */
  public void allModuleSetpoint() {
    frontLeft.setpoint();
    frontRight.setpoint();
    backLeft.setpoint();
    backRight.setpoint();
  }

  /** Gets Robot PID Setpoint */
  public double robotPIDSetpoint(char controller) {
    PIDController pid = pidMap.get(controller);
    return pid.getSetpoint();
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
}
