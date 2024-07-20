// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveModule;
import frc.robot.util.TunableNumber;

public class PIDTuning extends Command {
  SwerveModule module;
  private final PIDController pid;
  private final double setpoint;
  private TunableNumber tunableKP = new TunableNumber("Steer/kP");
  private TunableNumber tunableKI = new TunableNumber("Steer/kI");
  private TunableNumber tunableKD = new TunableNumber("Steer/kD");

  public PIDTuning(SwerveModule module, double setpoint) {
    this.module = module;
    pid = new PIDController(0.001, 0, 0);
    this.setpoint = setpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("PIDTuning Start");
    pid.setSetpoint(setpoint);
    pid.enableContinuousInput(0, 360);
    tunableKP.setDefault(0);
    tunableKI.setDefault(0);
    tunableKD.setDefault(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentkP = tunableKP.get();
    double currentkI = tunableKI.get();
    double currentkD = tunableKP.get();
/*
    if (currentkP != pid.getP()
    || currentkI != pid.getI()
    || currentkP != pid.getD()) {
      pid.setP(currentkP);
      pid.setI(currentkI);
      pid.setD(currentkD);
    }
*/
    double speed = pid.calculate(module.getAbsoluteEncoderDeg());
    if (speed < 0.1 && speed > -0.1) {
      module.setTurningSpeed(speed);
    } else {
      module.setTurningSpeed(0.1);
    }
    SmartDashboard.putNumber("Error", pid.getPositionError());
    SmartDashboard.putNumber("Output Speed", speed);
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
