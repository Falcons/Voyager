// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveModule;
import frc.robot.util.TunableNumber;

public class PIDTuning extends Command {
  SwerveModule module;
  private final double setpoint;

  public PIDTuning(SwerveModule module, double setpoint) {
    this.module = module;
    this.setpoint = setpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("PIDTuning Start");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = module.turningPIDCalculate(module.getAbsoluteEncoderDeg());//pid.calculate(module.getAbsoluteEncoderDeg());
    if (speed < 0.5 && speed > -0.5) {
      module.setTurningSpeed(speed);
    } else {
      module.setTurningSpeed(speed);
      System.out.println("Above 50%");
    }
    SmartDashboard.putNumber("Output Speed", speed);
    SmartDashboard.putNumber("Error", module.getPIDError());
    SmartDashboard.putNumber("Setpoint", module.getPIDsetpoint());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    module.stop();
    System.out.println("PIDTuning End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
