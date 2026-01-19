package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
    private final SparkMax driveMotor;
    private final SparkMax steerMotor;

    // ✅ Built-in SparkMax/NEO relative encoder (motor rotations/RPM)
    private final RelativeEncoder driveEncoder;

    // ✅ CANcoder absolute angle (steer)
    private final CANcoder angleCoder;

    private final PIDController turningPID;

    private final boolean angleCoderReversed;
    private final double angleOffsetRad;

    private final int driveMotorSign;
    private final int steerMotorSign;

    public SwerveModule(
            int driveMotorId,
            int steerMotorId,
            int cancoderId,
            boolean driveMotorReversed,
            boolean steerMotorReversed,
            boolean angleCoderReversed,
            double angleOffsetRad
    ) {
        this.angleCoder = new CANcoder(cancoderId);
        this.angleCoderReversed = angleCoderReversed;
        this.angleOffsetRad = angleOffsetRad;

        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        steerMotor = new SparkMax(steerMotorId, MotorType.kBrushless);

        this.driveMotorSign = driveMotorReversed ? -1 : 1;
        this.steerMotorSign = steerMotorReversed ? -1 : 1;


        driveEncoder = driveMotor.getEncoder();

        angleCoder.getConfigurator().apply(new CANcoderConfiguration());

        turningPID = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    // ✅ Absolute steer angle in radians (wrapped)
    public double getAngleRad() {
        // Phoenix 6 absolute position is in rotations
        double rotations = angleCoder.getAbsolutePosition().getValueAsDouble();
        double angleRad = rotations * 2.0 * Math.PI;

        angleRad -= angleOffsetRad;
        if (angleCoderReversed) angleRad *= -1.0;

        return MathUtil.angleModulus(angleRad);
    }

    public SwerveModuleState getState() {
        // SparkMax velocity is RPM by default
        double wheelMps = driveEncoder.getVelocity() * ModuleConstants.kDriveEncoderRPM2MeterPerSec;
        return new SwerveModuleState(wheelMps, new Rotation2d(getAngleRad()));
    }

    public SwerveModulePosition getPosition() {
        // SparkMax position is motor rotations by default
        double wheelMeters = driveEncoder.getPosition() * ModuleConstants.kDriveEncoderRot2Meter;
        return new SwerveModulePosition(wheelMeters, new Rotation2d(getAngleRad()));
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0.0);
        // ❌ Never “zero” the CANcoder position. Use offsets instead.
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // Prevent jitter when stopped
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        Rotation2d currentAngle = new Rotation2d(getAngleRad());

        // ✅ New WPILib pattern: copy then optimize (mutates)
        SwerveModuleState optimized = new SwerveModuleState(
                desiredState.speedMetersPerSecond,
                desiredState.angle
        );
        optimized.optimize(currentAngle);

        // Drive as percent output
        double driveOutput = optimized.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond;

        // Steer PID (measurement=current, setpoint=target)
        double turnOutput = turningPID.calculate(
                currentAngle.getRadians(),
                optimized.angle.getRadians()
        );

        // Keep steering from going nuclear
        turnOutput = MathUtil.clamp(turnOutput, -1.0, 1.0);

        driveMotor.set(driveOutput * driveMotorSign);
        steerMotor.setVoltage(steerMotorSign * turnOutput * 12.0);
    }

    public void stop() {
        driveMotor.set(0.0);
        steerMotor.set(0.0);
    }
}