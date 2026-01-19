package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoyStickCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;

    private final Supplier<Double> xSpdFunction;
    private final Supplier<Double> ySpdFunction;
    private final Supplier<Double> turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;

    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;
    private final SlewRateLimiter turningLimiter;

    public SwerveJoyStickCommand(
            SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction,
            Supplier<Double> ySpdFunction,
            Supplier<Double> turningSpdFunction,
            Supplier<Boolean> fieldOrientedFunction
    ) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;

        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        // 1) Read raw joystick axes (typically -1..1)
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();

        // 2) Deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        // Optional: square inputs for finer control near center (keeps sign!)
        // Comment these out if you want a more linear feel.
        xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
        ySpeed = Math.copySign(ySpeed * ySpeed, ySpeed);
        turningSpeed = Math.copySign(turningSpeed * turningSpeed, turningSpeed);

        // 3) Slew-rate limit + scale into real units
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        // 4) Build chassis speeds (field-relative or robot-relative)
        ChassisSpeeds speeds;

        if (fieldOrientedFunction.get()) {
            // ✅ Use the subsystem's Rotation2d (already in degrees -> Rotation2d handled correctly)
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed,
                    ySpeed,
                    turningSpeed,
                    swerveSubsystem.getRotation2d()
            );
        } else {
            speeds = new ChassisSpeeds(
                    xSpeed,
                    ySpeed,
                    turningSpeed
            );
        }

        // 5) Send to drivetrain
        swerveSubsystem.drive(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}