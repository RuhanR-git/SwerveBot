package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

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
        return new SwerveModulePosition(moduleCoder.getPosition().getValueAsDouble(), new Rotation2d(getAbsoluteAngleRad()));
    }

    public SwerveModuleState getState() 
    {
        return new SwerveModuleState(moduleCoder.getVelocity().getValueAsDouble(), new Rotation2d(getAbsoluteAngleRad()));
    }
    

    public void resetEncoders()
    {
        moduleCoder.setPosition(0.0); // Reset CANcoder position to 0
    }

    public double getAbsoluteAngleRad()
    {
        double rotations = moduleCoder.getPosition().getValueAsDouble();
        return rotations * 2.0 * Math.PI;
    }


    public void setDesiredState(SwerveModuleState desiredState) {
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
    
        desiredState.optimize(new Rotation2d(getAbsoluteAngleRad()));
    
        double turnOutput = turningPIDController.calculate(getAbsoluteAngleRad(), desiredState.angle.getRadians());
    
        driveMotor.set(
            desiredState.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond
        );
        steerMotor.set(turnOutput);
    }
    

    public void stop() 
    {
        driveMotor.set(0.0);
        steerMotor.set(0.0);
    }
}