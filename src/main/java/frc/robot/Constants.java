package frc.robot;

import edu.wpi.first.math.geometry.Translation2d; // WPILib 2D Translation class

import edu.wpi.first.math.kinematics.SwerveDriveKinematics; // WPILib Swerve Drive Kinematics class

import edu.wpi.first.math.trajectory.TrapezoidProfile; // WPILib Trapezoid Profile class

import edu.wpi.first.math.util.Units; // WPILib Units conversion class

// Constants.java is the "single source of truth" for robot numbers.
// It stores hardware IDs, robot dimensions, and tuning values in one place
// so you don't scatter magic numbers throughout the code.
//
// *Always reference set values here instead of within subsystem and command files
public final class Constants
{
    // Constants used inside a single swerve module (one wheel).
    public static final class ModuleConstants
    {
        // Wheel size (in meters). 0.0508m = 2 inches radius.
        public static final double kWheelRadiusMeters = 0.0508;

        // Diameter and circumference are derived values (so we don't type them wrong elsewhere).
        public static final double kWheelDiameterMeters = ModuleConstants.kWheelRadiusMeters * 2;
        public static final double kWheelCircumferenceMeters = ModuleConstants.kWheelDiameterMeters * Math.PI;

        // Gear ratios:
        // These convert motor rotations into wheel rotations (and steering rotations).
        // Example idea: if ratio were 1/6.75, the wheel turns 1 time for every 6.75 motor turns.
        public static final double kDriveMotorGearRatio = 1.0 / 6.75;
        public static final double kSteerMotorGearRatio = 1.0 / 6.75;

        // Conversion factors to turn encoder readings into real units.
        // - drive encoder: rotations -> meters traveled
        // - steer encoder: rotations -> radians turned
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kSteerEncoderRot2Rad = kSteerMotorGearRatio * 2 * Math.PI;

        // Encoder velocity conversions:
        // SparkMax gives RPM, so divide by 60 to get rotations per second.
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kSteerEncoderRot2Rad / 60;

        // Steering PID "P" gain used in SwerveModule turningPID.
        public static final double kPTurning = 0.0;

        // Steering PID "I" gain used in Swervemodule turning PID.
        public static final double kITurning = 0.0;

        //Steering PID "D" gain used in SwerveModule turning PID.
        public static final double kDTurning = 0.0;
    }
    
    // Extra steering P value (not currently used in the files you sent).
    // Keeping it here is fine if another file uses it.
    public static final double kPSteer = 0.5;

    // Constants that describe the full drivetrain: geometry, hardware IDs, and speed limits.
    public static final class DriveConstants
    {
        // Track width = distance between left and right wheels (side-to-side).
        public static final double kTrackWidth = Units.inchesToMeters(21);

        // Wheel base = distance between front and back wheels.
        public static final double kWheelBase = Units.inchesToMeters(25.5);

        // Kinematics describes where each module is located relative to the robot center.
        // WPILib uses these positions to convert:
        // - ChassisSpeeds (robot motion request) -> SwerveModuleState[] (wheel commands)
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics
        (
            // Translation2d(x, y) is the module position in meters from the robot center:
            // +x = forward, +y = left (WPILib coordinate convention)
            new Translation2d(kWheelBase / 2,  kTrackWidth / 2),  // Front Left
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),  // Front Right
            new Translation2d(-kWheelBase / 2,  kTrackWidth / 2), // Back Left
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)  // Back Right
        );
        
        // USB port for the driver controller on the Driver Station laptop.
        public static final int kDriverControllerPort = 0;

        // Motor CAN IDs for each module.
        // Each module has:
        // - a steer motor (rotates the wheel)
        // - a drive motor (spins the wheel)
        public static final int kFrontLeftSteerMotorPort = 1;
        public static final int kFrontLeftDriveMotorPort = 2;
        public static final int kBackRightSteerMotorPort = 3;
        public static final int kBackRightDriveMotorPort = 4;
        public static final int kFrontRightSteerMotorPort = 5;
        public static final int kFrontRightDriveMotorPort = 6;
        public static final int kBackLeftSteerMotorPort = 7;
        public static final int kBackLeftDriveMotorPort = 8;
        
        // If a motor spins the wrong direction, flip it here instead of swapping wires.
        public static final boolean kFrontLeftDriveMotorReversed = false;
        public static final boolean kFrontLeftSteerMotorReversed = false;
        public static final boolean kFrontRightDriveMotorReversed = false;
        public static final boolean kFrontRightSteerMotorReversed = false;
        public static final boolean kBackLeftDriveMotorReversed = false;
        public static final boolean kBackLeftSteerMotorReversed = false;
        public static final boolean kBackRightDriveMotorReversed = false;
        public static final boolean kBackRightSteerMotorReversed = false;

        // Encoder reversal flags (not used in the files you sent yet, but may be used elsewhere).
        public static final boolean kFrontLeftEncoderReversed = false;
        public static final boolean kBackLeftEncoderReversed = false;
        public static final boolean kFrontRightEncoderReversed = false;
        public static final boolean kBackRightSteerEncoderReversed = false;

        // CAN IDs for the absolute encoders (CANcoders) on each module steering axis.
        public static final int kFrontLeftCANcoderId  = 1;
        public static final int kFrontRightCANcoderId = 2;
        public static final int kBackLeftCANcoderId   = 3;
        public static final int kBackRightCANcoderId  = 4;

        // These look like older/alternate encoder ports (not used in the swerve files you sent).
        public static final int kFrontLeftDriveModuleCoderPort = 1;
        public static final int kBackLeftDriveModuleCoderPort = 2;
        public static final int kFrontRightDriveModuleCoderPort = 3;
        public static final int kBackRightDriveModuleCoderPort = 4;

        // CAN ID for the gyro (Pigeon2).
        public static final int kPigeonId = 69;

        // If a CANcoder reads angle backwards, flip it here.
        public static final boolean kFrontLeftModuleCoderReversed = false;
        public static final boolean kBackLeftModuleCoderReversed = false;
        public static final boolean kFrontRightModuleCoderReversed = false;
        public static final boolean kBackRightModuleCoderReversed = false;

        // Each module's "zero" offset (radians).
        // You measure this when calibrating so the robot knows what angle is "straight".
        public static final double kFrontLeftModuleCoderOffsetRad = 0.273;
        public static final double kBackLeftModuleCoderOffsetRad = 0.273;
        public static final double kFrontRightModuleCoderOffsetRad = -0.273;
        public static final double kBackRightModuleCoderOffsetRad = -0.273;

        // Physical max capabilities of the drivetrain (used for scaling and limiting).
        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        // Teleop limits: we intentionally cap speed lower than physical max for easier driving.
        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
        kPhysicalMaxAngularSpeedRadiansPerSecond / 4;

        // These are used by SlewRateLimiter to limit how fast commands can change.
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    // Constants for autonomous motion (trajectory following).
    public static final class AutoConstants 
    {
        // Auto speed limits (usually lower than physical max for stability).
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
        DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;

        // Auto acceleration limits.
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;

        // PID gains for driving to a pose (x, y, and heading).
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        // Constraints for turning control (how fast we can rotate and how fast we can accelerate rotation).
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints
        (
            kMaxAngularSpeedRadiansPerSecond,
            kMaxAngularAccelerationRadiansPerSecondSquared
        );
    }

    // "OI" = Operator Interface (driver controls).
    // This section is about joystick ports, axis numbers, button numbers, and deadbands.
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
    
        // Deadband removes small joystick drift near the center.
        public static final double kDeadband = 0.5;
    }    
}