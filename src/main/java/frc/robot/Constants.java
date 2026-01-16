package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

import edu.wpi.first.math.util.Units;

public final class Constants
{
    public static final class ModuleConstants
    {
        public static final double kWheelRadiusMeters = 0.0508;
        public static final double kWheelDiameterMeters = ModuleConstants.kWheelRadiusMeters * 2;
        public static final double kWheelCircumferenceMeters = ModuleConstants.kWheelDiameterMeters * Math.PI;
        public static final double kDriveMotorGearRatio = 1 / 6.75;
        public static final double kSteerMotorGearRatio = 1 / 6.75;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kSteerEncoderRot2Rad = kSteerMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kSteerEncoderRot2Rad / 60;
        public static final double kPTurning = 0.5;
    }
    
    public static final double kPSteer = 0.5;

    public static final class DriveConstants
    {
        public static final double kTrackWidth = Units.inchesToMeters(21);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(25.5);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics
        (
            new Translation2d(kWheelBase / 2,  kTrackWidth / 2),  // Front Left
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),  // Front Right
            new Translation2d(-kWheelBase / 2,  kTrackWidth / 2), // Back Left
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)  // Back Right
        );
        
        public static final int kDriverControllerPort = 0;

        public static final int kFrontLeftDriveMotorPort = 2;
        public static final int kFrontLeftSteerMotorPort = 1;
        public static final int kBackRightDriveMotorPort = 4;
        public static final int kBackRightSteerMotorPort = 3;
        public static final int kFrontRightDriveMotorPort = 6;
        public static final int kFrontRightSteerMotorPort = 5;
        public static final int kBackLeftDriveMotorPort = 8;
        public static final int kBackLeftSteerMotorPort = 7;

        public static final boolean kFrontLeftDriveMotorReversed = false;
        public static final boolean kFrontLeftSteerMotorReversed = false;
        public static final boolean kFrontRightDriveMotorReversed = false;
        public static final boolean kFrontRightSteerMotorReversed = false;
        public static final boolean kBackLeftDriveMotorReversed = false;
        public static final boolean kBackLeftSteerMotorReversed = false;
        public static final boolean kBackRightDriveMotorReversed = false;
        public static final boolean kBackRightSteerMotorReversed = false;

        public static final boolean kFrontLeftEncoderReversed = false;
        public static final boolean kBackLeftEncoderReversed = false;
        public static final boolean kFrontRightEncoderReversed = false;
        public static final boolean kBackRightSteerEncoderReversed = false;

        public static final int kFrontLeftCANcoderId  = 1;
        public static final int kFrontRightCANcoderId = 2;
        public static final int kBackLeftCANcoderId   = 3;
        public static final int kBackRightCANcoderId  = 4;

        public static final int kFrontLeftDriveModuleCoderPort = 1;
        public static final int kBackLeftDriveModuleCoderPort = 2;
        public static final int kFrontRightDriveModuleCoderPort = 3;
        public static final int kBackRightDriveModuleCoderPort = 4;

        public static final int kPigeonId = 69;

        public static final boolean kFrontLeftModuleCoderReversed = false;
        public static final boolean kBackLeftModuleCoderReversed = false;
        public static final boolean kFrontRightModuleCoderReversed = false;
        public static final boolean kBackRightModuleCoderReversed = false;

        public static final double kFrontLeftModuleCoderOffsetRad = 0;
        public static final double kBackLeftModuleCoderOffsetRad = 0;
        public static final double kFrontRightModuleCoderOffsetRad = 0;
        public static final double kBackRightModuleCoderOffsetRad = 0;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
        kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public static final class AutoConstants 
    {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
        DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints
            (
                kMaxAngularSpeedRadiansPerSecond,
                kMaxAngularAccelerationRadiansPerSecondSquared
            );
    }

    public static final class OIConstants {
        public static final int kDriverJoystickPort = 0;
    
        // Xbox mapping when using wpilib Joystick raw axes:
        // Left stick X = 0
        // Left stick Y = 1
        // Right stick X = 4
        public static final int kDriverXAxis = 0;      // strafe
        public static final int kDriverYAxis = 1;      // forward/back
        public static final int kDriverRotAxis = 4;    // rotation
    
        // Buttons are typically 1-indexed (A=1, B=2, X=3, Y=4).
        // Pick the one you actually want for field oriented toggle/hold.
        public static final int kDriverFieldOrientedButtonIdx = 1;
    
        public static final double kDeadband = 0.25;
    }    
}