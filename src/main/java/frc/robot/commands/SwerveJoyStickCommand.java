package frc.robot.commands; // Package for robot commands

// Java "interface" type that represents a function you can call later.
// In Python terms: like passing a function reference / lambda that returns a value.
import java.util.function.Supplier;

// WPILib helper that limits how fast a value can change (prevents jerky acceleration).
import edu.wpi.first.math.filter.SlewRateLimiter;

// WPILib class that represents the robot's desired motion:
// vx (forward), vy (left), and omega (turn rate).
import edu.wpi.first.math.kinematics.ChassisSpeeds;

// Base class for a WPILib "Command" (the scheduler runs these during teleop/auto).
import edu.wpi.first.wpilibj2.command.Command;

// Project-specific imports (subsystems, constants).
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

// This command reads joystick inputs every loop and tells the swerve subsystem how to drive.
// Think of it like: "teleop drive logic" that runs continuously while scheduled.
public class SwerveJoyStickCommand extends Command {

    // The subsystem this command controls (drivetrain / swerve).
    // In WPILib, Commands "require" subsystems so the scheduler prevents conflicts.
    private final SwerveSubsystem swerveSubsystem;

    // Suppliers are "live getters" for values.
    // Instead of storing joystick values once, we store *functions* that we can call every loop.
    // Python analogy: passing in something like lambda: joystick.getX()
    private final Supplier<Double> xSpdFunction;
    private final Supplier<Double> ySpdFunction;
    private final Supplier<Double> turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;

    // Slew rate limiters smooth out sudden joystick changes.
    // Without these, the robot might accelerate too abruptly and feel hard to control.
    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;
    private final SlewRateLimiter turningLimiter;

    // Constructor: runs once when the command is created.
    // It "injects" the subsystem and the joystick input suppliers.
    public SwerveJoyStickCommand(
            SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction,
            Supplier<Double> ySpdFunction,
            Supplier<Double> turningSpdFunction,
            Supplier<Boolean> fieldOrientedFunction
    ) {
        // Save references so execute() can use them later.
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;

        // Configure how quickly we're allowed to ramp joystick commands.
        // These constants live in DriveConstants so tuning is centralized.
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

        // Tells WPILib: "this command uses the swerveSubsystem."
        // That prevents another command from driving the same subsystem at the same time.
        addRequirements(swerveSubsystem);
    }

    // execute() is called repeatedly by the Command Scheduler (usually every ~20ms).
    @Override
    public void execute() {
        // 1) Read raw joystick axes (typically -1..1)
        // We call .get() because each Supplier is a function that returns the current value.
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();

        // 2) Deadband
        // Joysticks often "drift" slightly even when centered.
        // Deadband forces small values to 0 so the robot doesn't creep.
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        // Optional: square inputs for finer control near center (keeps sign!)
        // Squaring makes small joystick movements even smaller (more precision),
        // while still allowing full output at the ends.
        // Math.copySign(...) keeps the original direction (positive/negative).
        // Comment these out if you want a more linear feel.
        xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
        ySpeed = Math.copySign(ySpeed * ySpeed, ySpeed);
        turningSpeed = Math.copySign(turningSpeed * turningSpeed, turningSpeed);

        // 3) Slew-rate limit + scale into real units
        // First: smooth the joystick values (rate limiting)
        // Then: multiply by max speed constants to convert from [-1..1] into real units.
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        // 4) Build chassis speeds (field-relative or robot-relative)
        // ChassisSpeeds is WPILib's standard way to represent "how we want the robot to move."
        ChassisSpeeds speeds;

        if (fieldOrientedFunction.get()) {
            // Field-oriented driving:
            // - "forward" always means away from your driver station wall,
            //   even if the robot is rotated.
            // WPILib needs the robot's current heading (Rotation2d) to rotate field inputs
            // into robot-relative commands for the swerve modules.
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed,
                    ySpeed,
                    turningSpeed,
                    swerveSubsystem.getRotation2d()
            );
        } else {
            // Robot-oriented driving:
            // - forward/left are based on where the robot is facing right now.
            speeds = new ChassisSpeeds(
                    xSpeed,
                    ySpeed,
                    turningSpeed
            );
        }

        // 5) Send to drivetrain
        // The subsystem will take these chassis speeds and convert them to individual module states.
        swerveSubsystem.drive(speeds);
    }

    // end() runs once when the command finishes or gets interrupted (canceled).
    @Override
    public void end(boolean interrupted) {
        // Safety: stop the modules so the robot doesn't keep rolling.
        swerveSubsystem.stopModules();
    }

    // This command is intended to run the whole time during teleop,
    // so it never ends on its own.
    @Override
    public boolean isFinished() {
        return false;
    }
}