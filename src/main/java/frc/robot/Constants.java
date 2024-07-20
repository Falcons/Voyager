// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public final class Constants {

    public static final class ModuleConstants {
        public static final double freeSpeedMpS = 4.60248;
        public static final double voltToDegree = 360.0 / 3.3;
        public static final double driveRotToWheelRot = 14.0/50.0 * 27.0/17.0 * 15.0/45.0;
        public static final double driveRPMToMetresPerSecond = driveRotToWheelRot / 60.0;
        public static final double turningRotToWheelDegree = 1.0 / (150.0 / 7.0) * 360;
        public static final double turningRPMToDegreePerSecond = turningRotToWheelDegree / 60.0;

        public static final int frontLeftDriveCANID = 4;
        public static final int frontLeftTurningCANID = 3;
        public static final double frontLeftAbsoluteOffset = 221.86;

        public static final int frontRightDriveCANID = 7;
        public static final int frontRightTurningCANID = 8;
        public static final double frontRightAbsoluteOffset = 152.70;

        public static final int backLeftDriveCANID = 2;
        public static final int backLeftTurningCANID = 1;
        public static final double backLeftAbsoluteOffset = 72.73;

        public static final int backRightDriveCANID = 5;
        public static final int backRightTurningCANID = 6;
        public static final double backRightAbsoluteOffset = 95.95;
    }

    public static final class Conversion {
        public static final double degToRad = Math.PI / 180.0;
    }

    public static final class Misc {
        public static final boolean tuningMode = true;
    }
}