package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
    public final String moduleName;
    private final CANSparkMax driveMotor, turningMotor;
    private final RelativeEncoder driveEncoder, turningEncoder;

    private final PIDController turningPID;

    private final SparkAnalogSensor absEncoder;
    private final double absEncoderOffset;

    public SwerveModule(String name, int driveMotorID, int turningMotorID, boolean reversed, double offsetDegrees) {
        this.moduleName = name;
        
        this.driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        driveMotor.restoreFactoryDefaults();

        this.turningMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);
        turningMotor.restoreFactoryDefaults();
        turningMotor.setInverted(reversed);

        this.driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(ModuleConstants.driveMotorRotToMetre);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.driveMotorRPMToMetresPerSecond);

        this.turningEncoder = turningMotor.getEncoder();
        turningEncoder.setPositionConversionFactor(ModuleConstants.turningRotToWheelDegree);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.turningRPMToDegreePerSecond);

        this.absEncoder = turningMotor.getAnalog();
        this.absEncoderOffset = Units.degreesToRadians(offsetDegrees);

        absEncoder.setPositionConversionFactor(ModuleConstants.voltToRad);

        //kI value is only this high due to max integrator range
        turningPID = new PIDController(0.004, 0.05, 0);
        turningPID.enableContinuousInput(-180, 180);
        turningPID.setTolerance(0.1);
        turningPID.setIntegratorRange(-0.01, 0.01);
        turningEncoder.setPosition(getAbsEncoderDeg());
        
        SmartDashboard.putData("TurningPID/" + this.moduleName, this.turningPID);

        resetEncoders();
    }

    /** Stops Drive and Turning Motors */
    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

// Integrated Drive Encoder

    /** @return Drive Encoder Postion in Metres */
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    /** @return Drive Encoder position in Metres per Second */
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

// Integrated Turning Encoder

    /** @return Raw Integrated Turning Encoder */
    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    /** @return Integrated Turning Encoder in degrees (-180 to 180) CCW+ */
    public double getTurningEncoderDegree() {
        return Math.IEEEremainder(turningEncoder.getPosition(), 360);
      } 
    

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

// Absolute Encoder

    /** @return Abs position in correct units, incorrect range */
    public double rawPositionWithOffset() {
        return absEncoder.getPosition() - absEncoderOffset;
    }

    /** @return Absolute Encoder in radians (-Pi to Pi) CCW+ */
    public double getAbsEncoderRad() {
        if (rawPositionWithOffset() < -Math.PI) {
            return rawPositionWithOffset() + 2 * Math.PI;

        } else if (rawPositionWithOffset() > Math.PI){
            return rawPositionWithOffset() - 2 * Math.PI;

        } else {
            return rawPositionWithOffset();
        }
    }

    /** @return Absolute Encoder in degrees (-180 to 180) CCW+ */
    public double getAbsEncoderDeg() {
        return getAbsEncoderRad() * 180.0 / Math.PI;
    }

    /** Resets Drive encoders and matches Turning encoders with absolute */
    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsEncoderDeg());
    }

// SwerveModuleState

    /** @return Current Speed and Angle of Module */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAbsEncoderRad()));
    }

    /** Sets Module to specified Speed and Angle */
    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / ModuleConstants.driveMaxSpeedMPS);
        turningMotor.set(turningPID.calculate(getAbsEncoderDeg(), state.angle.getDegrees()));
    }

// SwerveModulePosition

    /** @return Current Position and Angle of Module */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), getState().angle);
    }

//PID

    /** Turning Motor to Setpoint */
    public void pidTuning() {
        turningMotor.set(turningPID.calculate(getAbsEncoderDeg()));
    }

    /** Sets PID setpoint from Smartdashboard value */
    public void setpoint() {
        double stpt = SmartDashboard.getNumber("Module Setpoint", 0);
        turningPID.setSetpoint(stpt);
    }   

    /** Resets the previous error and the integral term. */
    public void pidReset() {
        turningPID.reset();
    }
}
