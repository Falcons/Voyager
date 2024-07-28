package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Conversion;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
    public final String moduleName;
    private final CANSparkMax driveMotor, turningMotor;
    private final RelativeEncoder driveEncoder, turningEncoder;

    private final PIDController turningPID;

    private final SparkAnalogSensor absEncoder;
    private final double absEncoderOffset;

    public SwerveModule(String name, int driveMotorID, int turningMotorID, boolean reversed, double offset) {
        this.moduleName = name;
        this.driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        driveMotor.restoreFactoryDefaults();

        this.turningMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);
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
        turningPID.enableContinuousInput(0, 360);
        turningPID.setTolerance(0.1);
        turningPID.setIntegratorRange(-0.01, 0.01);
        turningEncoder.setPosition(getAbsoluteEncoderDeg());

        SmartDashboard.putData("TurningPID/" + this.moduleName, this.turningPID);

        resetEncoders();
    }

    /**
     * 
     * @return Drive Encoder Postion in Metres
     */
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    /**
     * @return Raw Integrated Turning Encoder
     */
    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }
    /**
     * @return Integrated Turning Encoder in degrees (-180 to 180) CCW+
     */
    public double getTurningEncoderDegree() {
        return Math.IEEEremainder(turningEncoder.getPosition(), 360);
      } 
    
    /**
     * @return Drive Encoder position in Metres per Second
     */
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    /**
     * @return Absolute Encoder in degrees (-180 to 180) CCW+
     */
    public double getAbsoluteEncoderDeg() {
        if (absEncoder.getPosition() - absEncoderOffset < 180 && absEncoder.getPosition() > -180) {
            return absEncoder.getPosition() - absEncoderOffset;
        } else if (absEncoder.getPosition() < -180){
            return absEncoder.getPosition() - absEncoderOffset + 360;
        } else {
            return absEncoder.getPosition() - absEncoderOffset - 360;
        }
      }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderDeg());
    }

    public SwerveModuleState getState() {
        //angle must be in radians
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAbsoluteEncoderDeg() * Conversion.degToRad));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / ModuleConstants.driveFreeSpeedMPS);
        turningMotor.set(turningPID.calculate(getAbsoluteEncoderDeg(), state.angle.getDegrees()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), getState().angle);
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

    //PID Controller is already printed to dashboard, values can be changed live
    public void pidTuning() {
        turningMotor.set(turningPID.calculate(getAbsoluteEncoderDeg()));
    }

    public void pidReset() {
        turningPID.reset();
    }
}
