package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Conversion;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {
    private final CANSparkMax driveMotor, turningMotor;
    private final RelativeEncoder driveEncoder, turningEncoder;

    private final PIDController turningPID;

    private final SparkAnalogSensor absEncoder;
    private final double absEncoderOffset;

    public SwerveModule(int driveMotorID, int turningMotorID, boolean reversed, double offset) {
        this.driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        this.turningMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);

        driveMotor.restoreFactoryDefaults();
        turningMotor.restoreFactoryDefaults();

        turningMotor.setInverted(reversed);

        this.driveEncoder = driveMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(ModuleConstants.driveRotToMetre);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.driveRPMToMetresPerSecond);

        this.turningEncoder = turningMotor.getEncoder();
        
        turningEncoder.setPositionConversionFactor(ModuleConstants.turningRotToWheelDegree);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.turningRPMToDegreePerSecond);

        this.absEncoder = turningMotor.getAnalog();
        this.absEncoderOffset = offset;

        absEncoder.setPositionConversionFactor(ModuleConstants.voltToDegree);
        
        //kI value is only this high due to max integrator range
        turningPID = new PIDController(0.004, 0.05, 0);
        turningPID.enableContinuousInput(-180, 180);
        turningPID.setTolerance(0.1);
        turningPID.setIntegratorRange(-0.01, 0.01);
        
        turningEncoder.setPosition(getAbsoluteEncoderDeg());

        SmartDashboard.putData(turningPID);
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public void resetDriveEncoder() {
        driveEncoder.setPosition(0);
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public double getTurningEncoderDegree() {
        return Math.IEEEremainder(turningEncoder.getPosition(), 360);
      } 

    public double getSpeed() {
        return driveMotor.get();
    }
    
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRadians() {
        return absEncoder.getPosition() * Conversion.degToRad;
    }

    public double getAbsoluteEncoderDeg() {
        if (absEncoder.getPosition() - absEncoderOffset < 180 && absEncoder.getPosition() > -180) {
            return absEncoder.getPosition() - absEncoderOffset;
        } else if (absEncoder.getPosition() < -180){
            return absEncoder.getPosition() - absEncoderOffset + 360;
        } else {
            return absEncoder.getPosition() - absEncoderOffset - 360;
        }
      }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

    public void setDriveSpeedMpS(double speed) {
        driveMotor.set(speed / ModuleConstants.freeSpeedMpS);
    }

    public void setTurningSpeed(double speed) {
        turningMotor.set(speed);
    }

    public double turningPIDCalculate(double measurement) {
        return turningPID.calculate(measurement);
    }

    public void setPIDSetpoint(double setpoint) {
        turningPID.setSetpoint(setpoint);
    }

    public double getPIDsetpoint() {
        return turningPID.getSetpoint();
    }

    public double getPIDError() {
        return turningPID.getPositionError();
    }

    public boolean getAtSetpoint() {
        return turningPID.atSetpoint();
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderDeg());
    }
}
