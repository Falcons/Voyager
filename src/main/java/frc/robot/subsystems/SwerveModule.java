package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Conversion;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {
    private final CANSparkMax driveMotor, turningMotor;
    private final RelativeEncoder driveEncoder, turningEncoder;

    private final SparkAnalogSensor absEncoder;
    private final double absEncoderOffset;

    public SwerveModule(int driveMotorID, int turningMotorID, boolean reversed, double offset) {
        this.driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        this.turningMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);

        driveMotor.restoreFactoryDefaults();
        turningMotor.restoreFactoryDefaults();

        turningMotor.setInverted(reversed);

        this.driveEncoder = driveMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(ModuleConstants.driveRotToWheelRot);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.driveRPMToMetresPerSecond);

        this.turningEncoder = turningMotor.getEncoder();
        
        turningEncoder.setPositionConversionFactor(ModuleConstants.turningRotToWheelDegree);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.turningRPMToDegreePerSecond);

        this.absEncoder = turningMotor.getAnalog();
        this.absEncoderOffset = offset;

        absEncoder.setPositionConversionFactor(ModuleConstants.voltToDegree);

        turningEncoder.setPosition(getAbsoluteEncoderDeg());
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public double getTurningEncoderDegree() {
        //typecasting for integer division
        int divisor = (int)turningEncoder.getPosition() / 360;
        
        if (turningEncoder.getPosition() > 0) {
          return turningEncoder.getPosition() - divisor * 360;
        } else {
          return turningEncoder.getPosition() - (divisor - 1.0) * 360;
        }
      } 

    public double getSpeed() {
        return driveMotor.get();
    }
    
    //make constant for
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
        if (absEncoder.getPosition() - absEncoderOffset < 0) {
          return absEncoder.getPosition() + 360 - absEncoderOffset;
        } else {
          return absEncoder.getPosition() - absEncoderOffset;
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
}