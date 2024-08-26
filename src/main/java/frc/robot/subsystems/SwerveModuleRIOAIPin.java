package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogEncoder;
/** Test Class, unused */
public class SwerveModuleRIOAIPin extends SwerveModule {

    private AnalogEncoder absEncoderRIO;

    SwerveModuleRIOAIPin(String name, int driveMotorID, int turningMotorID, int absEncoderPort, boolean reversed, double offsetDegrees) {
        super(name, driveMotorID, turningMotorID, 0, reversed, offsetDegrees);

        this.absEncoderRIO = new AnalogEncoder(0);
    }

    @Override
    public double getAbsEncoderRaw() {
        return absEncoderRIO.get();
    }
}
