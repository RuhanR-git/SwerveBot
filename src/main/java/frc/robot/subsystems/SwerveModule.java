package frc.robot.subsystems; // Package for robot subsystems

import com.ctre.phoenix6.configs.CANcoderConfiguration; // CANcoder configuration class
import com.ctre.phoenix6.hardware.CANcoder; // CANcoder hardware class

import com.revrobotics.RelativeEncoder; // Rev Robotics encoder class (built-in to Spark Maxes)
import com.revrobotics.spark.SparkLowLevel.MotorType; // Motor type enum for Spark Max
import com.revrobotics.spark.SparkMax; // Spark Max motor controller class

import edu.wpi.first.math.MathUtil; // WPILib math functions
import edu.wpi.first.math.controller.PIDController; // WPILib PID controller class
import edu.wpi.first.math.geometry.Rotation2d; // WPILib rotation (angle) class
import edu.wpi.first.math.kinematics.SwerveModulePosition; // WPILib swerve module position class
import edu.wpi.first.math.kinematics.SwerveModuleState; // WPILib swerve module state class

// Constants for the robot (motor IDs, physical parameters, etc).
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

// One "swerve module" = one wheel that can:
// 1) drive (spin the wheel forward/backward)
// 2) steer (rotate the wheel to any angle)
//
// This class wraps the hardware (motors + sensors) and exposes simple methods like:
// - getState() / getPosition() for odometry
// - setDesiredState(...) to command speed + angle
public class SwerveModule {
    // Motor that drives the wheel (forward/backward)
    private final SparkMax driveMotor;

    // Motor that rotates the module (steering angle)
    private final SparkMax steerMotor;

    // Encoder built into the drive motor:
    // - position: how many motor rotations have happened
    // - velocity: how fast the motor is spinning (RPM)
    private final RelativeEncoder driveEncoder;

    // Absolute angle sensor on the steering axis.
    // "Absolute" means it knows the wheel angle even after reboot.
    private final CANcoder angleCoder;

    // PID controller that makes the steering motor turn to a target angle
    private final PIDController turningPID;

    // If the CANcoder is mounted in the opposite direction, flip the reading
    private final boolean angleCoderReversed;

    // Mechanical offset so "zero angle" matches how the module is physically mounted
    private final double angleOffsetRad;

    // Some motors may need their direction flipped depending on wiring / mounting.
    // We store +1 or -1 and multiply outputs by it.
    private final int driveMotorSign;
    private final int steerMotorSign;

    // Constructor: creates the module hardware and sets up sensors/controllers.
    public SwerveModule(
            int driveMotorId,
            int steerMotorId,
            int cancoderId,
            boolean driveMotorReversed,
            boolean steerMotorReversed,
            boolean angleCoderReversed,
            double angleOffsetRad
    ) {
        // Create the angle sensor (reads steering angle over CAN)
        this.angleCoder = new CANcoder(cancoderId);
        this.angleCoderReversed = angleCoderReversed;
        this.angleOffsetRad = angleOffsetRad;

        // Create the motors (NEOs are brushless)
        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        steerMotor = new SparkMax(steerMotorId, MotorType.kBrushless);

        // Convert booleans into +1 / -1 so we can flip motor direction cleanly
        this.driveMotorSign = driveMotorReversed ? -1 : 1;
        this.steerMotorSign = steerMotorReversed ? -1 : 1;

        // Grab the built-in encoder from the drive motor
        driveEncoder = driveMotor.getEncoder();

        // Apply default CANcoder configuration (good baseline settings)
        angleCoder.getConfigurator().apply(new CANcoderConfiguration());

        // Steering PID (P-only here). "P" is how hard we correct toward the target.
        turningPID = new PIDController(ModuleConstants.kPTurning, 0, 0);

        // Angles wrap around (π is the same direction as -π), so we tell PID to treat it as circular.
        turningPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    // Returns the module's steering angle in radians.
    // This is based on the CANcoder absolute position, plus our offset and optional reversal.
    public double getAngleRad() {
        // Phoenix 6 gives absolute position in "rotations" (1.0 = one full turn)
        double rotations = angleCoder.getAbsolutePosition().getValueAsDouble();

        // Convert rotations -> radians (2π radians in one rotation)
        double angleRad = rotations * 2.0 * Math.PI;

        // Apply the physical mounting offset so our "zero" is correct
        angleRad -= angleOffsetRad;

        // Flip sign if the sensor is reversed
        if (angleCoderReversed) angleRad *= -1.0;

        // Wrap angle to a standard range so it doesn't grow forever
        return MathUtil.angleModulus(angleRad);
    }

    // Returns current speed + angle for this module (used for logging and control).
    public SwerveModuleState getState() {
        // SparkMax velocity is in RPM, so we convert it into wheel meters/second.
        double wheelMps = driveEncoder.getVelocity() * ModuleConstants.kDriveEncoderRPM2MeterPerSec;

        // Rotation2d is WPILib's "angle object" (stores radians under the hood).
        return new SwerveModuleState(wheelMps, new Rotation2d(getAngleRad()));
    }

    // Returns distance traveled + current angle (used for odometry).
    public SwerveModulePosition getPosition() {
        // SparkMax position is in motor rotations, so we convert into wheel meters.
        double wheelMeters = driveEncoder.getPosition() * ModuleConstants.kDriveEncoderRot2Meter;

        return new SwerveModulePosition(wheelMeters, new Rotation2d(getAngleRad()));
    }

    // Resets the drive encoder distance to zero (useful when starting odometry).
    public void resetEncoders() {
        driveEncoder.setPosition(0.0);

        // Note: we do not reset the CANcoder angle here.
        // The CANcoder is an absolute sensor, and we handle "zeroing" with angleOffsetRad.
    }

    // Main control method: command this module to a desired wheel speed + steering angle.
    public void setDesiredState(SwerveModuleState desiredState) {
        // When the robot is basically stopped, small angle corrections can cause jitter.
        // So if we're not trying to drive, we just stop outputs.
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        // Current steering angle of the module
        Rotation2d currentAngle = new Rotation2d(getAngleRad());

        // WPILib can "optimize" a swerve command:
        // It may choose to rotate the wheel 180° and reverse wheel direction
        // because that can be faster than turning the module a long distance.
        SwerveModuleState optimized = new SwerveModuleState(
                desiredState.speedMetersPerSecond,
                desiredState.angle
        );
        optimized.optimize(currentAngle);

        // Drive motor output as a percentage (-1 to 1)
        // We scale desired m/s by the max physical speed to get a percent.
        double driveOutput = optimized.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond;

        // Steering motor output comes from PID:
        // - measurement = current angle
        // - setpoint    = target angle
        double turnOutput = turningPID.calculate(
                currentAngle.getRadians(),
                optimized.angle.getRadians()
        );

        // Safety clamp so steering can't command crazy values
        turnOutput = MathUtil.clamp(turnOutput, -1.0, 1.0);

        // Apply outputs to motors (including sign flips for reversed motors)
        driveMotor.set(driveOutput * driveMotorSign);

        // For steering, we use voltage control here (0 to 12V, signed).
        steerMotor.setVoltage(steerMotorSign * turnOutput * 12.0);
    }

    // Stops both motors for this module.
    public void stop() {
        driveMotor.set(0.0);
        steerMotor.set(0.0);
    }
}
