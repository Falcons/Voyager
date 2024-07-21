// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveModule;

public class DisplayValues extends Command {
  SwerveModule frontleft, frontright, backleft, backright;
  
  public DisplayValues(SwerveModule frontleft, 
    SwerveModule frontright, 
    SwerveModule backleft, 
    SwerveModule backright) {
    this.frontleft = frontleft;
    this.frontright = frontright;
    this.backleft = backleft;
    this.backright = backright;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("DisplayValues start");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Drive/FrontLeft", frontleft.getDrivePosition());
    SmartDashboard.putNumber("Drive/FrontRight", frontright.getDrivePosition());
    SmartDashboard.putNumber("Drive/BackLeft", backleft.getDrivePosition());
    SmartDashboard.putNumber("Drive/BackRight", backright.getDrivePosition());

    SmartDashboard.putNumber("Steer/FrontLeft", frontleft.getTurningEncoderDegree());
    SmartDashboard.putNumber("Steer/FrontRight", frontright.getTurningEncoderDegree());
    SmartDashboard.putNumber("Steer/BackLeft", backleft.getTurningEncoderDegree());
    SmartDashboard.putNumber("Steer/BackRight", backright.getTurningEncoderDegree());

    SmartDashboard.putNumber("Abs/FrontLeft", frontleft.getAbsoluteEncoderDeg());
    SmartDashboard.putNumber("Abs/FrontRight", frontright.getAbsoluteEncoderDeg());
    SmartDashboard.putNumber("Abs/BackLeft", backleft.getAbsoluteEncoderDeg());
    SmartDashboard.putNumber("Abs/BackRight", backright.getAbsoluteEncoderDeg());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
