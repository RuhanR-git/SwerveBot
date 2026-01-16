package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.RobotController;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;


public class SwerveModule 
{
    private final SparkMax driveMotor;
    private final SparkMax steerMotor;

    private final CANcoder moduleCoder;

    private final PIDController turningPIDController;

    private final boolean moduleCoderReversed;
    private final double moduleCoderOffsetRad;
    
    public SwerveModule(int driveMotorId, int steerMotorId, int cancoderId, boolean moduleCoderReversed, double moduleCoderOffsetRad) 
    {
        this.moduleCoder = new CANcoder(cancoderId);

        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        steerMotor = new SparkMax(steerMotorId, MotorType.kBrushless);

        moduleCoder.getConfigurator().apply(new CANcoderConfiguration() {{}});

        turningPIDController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        this.moduleCoderReversed = moduleCoderReversed;
        this.moduleCoderOffsetRad = moduleCoderOffsetRad;

        resetEncoders();
    }

    public SwerveModulePosition getPosition()
    {
        // Spark MAX integrated encoder (motor rotations)
        // NOTE: Depending on your REVLib version, getEncoder() may require an import.
        double motorRotations = driveMotor.getEncoder().getPosition();

        // Convert motor rotations -> wheel meters (uses gear ratio in ModuleConstants)
        double meters = motorRotations * ModuleConstants.kDriveEncoderRot2Meter;

        // Module angle from your analog absolute encoder (already calibrated with offset)
        Rotation2d angle = new Rotation2d(getAbsoluteEncoderRad());

        return new SwerveModulePosition(meters, angle);
    }

    public SwerveModuleState getState()
    {
        // Spark MAX integrated encoder velocity (RPM in most REV APIs)
        double motorRpm = driveMotor.getEncoder().getVelocity();

        // Convert RPM -> m/s
        double metersPerSecond = motorRpm * ModuleConstants.kDriveEncoderRPM2MeterPerSec;

        Rotation2d angle = new Rotation2d(getAbsoluteEncoderRad());

        return new SwerveModuleState(metersPerSecond, angle);
    }

    public double getAbsoluteEncoderRad() 
    {
        double angle = moduleCoder.getSupplyVoltage().getValueAsDouble() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= moduleCoderOffsetRad;
        return angle * (moduleCoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders()
    {
        // Zero drive distance
        driveMotor.getEncoder().setPosition(0.0);

        // We do NOT "zero" the absolute encoder; it is absolute by nature.
        // If you later seed a relative steer encoder, that seeding happens here.
    }


    public void setDesiredState(SwerveModuleState state)
    {
        Rotation2d currentAngle = new Rotation2d(getAbsoluteEncoderRad());
        state.optimize(currentAngle);

        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

        // turningPIDController wants radians
        double turnOutput = turningPIDController.calculate(currentAngle.getRadians(), state.angle.getRadians());
        steerMotor.set(turnOutput);
    }

    public void stop() 
    {
        driveMotor.set(0.0);
        steerMotor.set(0.0);
    }
}