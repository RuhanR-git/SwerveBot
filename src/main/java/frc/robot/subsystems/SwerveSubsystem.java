package frc.robot.subsystems; // Package for robot subsystems

import com.ctre.phoenix6.hardware.Pigeon2; // Pigeon2 gyro class

import edu.wpi.first.math.geometry.Pose2d; // Robot pose (x, y, heading)
import edu.wpi.first.math.geometry.Rotation2d; // WPILib rotation (angle) class
import edu.wpi.first.math.kinematics.ChassisSpeeds; // Robot motion request class
import edu.wpi.first.math.kinematics.SwerveDriveKinematics; // Swerve drive kinematics class
import edu.wpi.first.math.kinematics.SwerveDriveOdometry; // Swerve drive odometry class
import edu.wpi.first.math.kinematics.SwerveModulePosition; // WPILib swerve module position class
import edu.wpi.first.math.kinematics.SwerveModuleState; // WPILib swerve module state class

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; // WPILib SmartDashboard for telemetry
import edu.wpi.first.wpilibj2.command.SubsystemBase; // Base class for WPILib subsystems

import frc.robot.Constants.DriveConstants; // Drive constants

// Subsystem = the "owner" of the drivetrain hardware.
// Commands (like your joystick command) call methods in here (like drive(...)).
//
// This subsystem holds:
// - 4 SwerveModule objects (one per wheel)
// - a gyro (robot heading)
// - odometry (keeps track of robot position on the field)
public class SwerveSubsystem extends SubsystemBase {

    // Each wheel module is created with IDs + configuration from Constants.
    // This keeps all the "wiring numbers" in one place (DriveConstants).
    private final SwerveModule frontLeft = new SwerveModule(
        DriveConstants.kFrontLeftDriveMotorPort,
        DriveConstants.kFrontLeftSteerMotorPort,
        DriveConstants.kFrontLeftCANcoderId,
        DriveConstants.kFrontLeftDriveMotorReversed,
        DriveConstants.kFrontLeftSteerMotorReversed,
        DriveConstants.kFrontLeftModuleCoderReversed,
        DriveConstants.kFrontLeftModuleCoderOffsetRad
    );

    private final SwerveModule frontRight = new SwerveModule(
        DriveConstants.kFrontRightDriveMotorPort,
        DriveConstants.kFrontRightSteerMotorPort,
        DriveConstants.kFrontRightCANcoderId,
        DriveConstants.kFrontRightDriveMotorReversed,
        DriveConstants.kFrontRightSteerMotorReversed,
        DriveConstants.kFrontRightModuleCoderReversed,
        DriveConstants.kFrontRightModuleCoderOffsetRad
    );

    private final SwerveModule backLeft = new SwerveModule(
        DriveConstants.kBackLeftDriveMotorPort,
        DriveConstants.kBackLeftSteerMotorPort,
        DriveConstants.kBackLeftCANcoderId,
        DriveConstants.kBackLeftDriveMotorReversed,
        DriveConstants.kBackLeftSteerMotorReversed,
        DriveConstants.kBackLeftModuleCoderReversed,
        DriveConstants.kBackLeftModuleCoderOffsetRad
    );

    private final SwerveModule backRight = new SwerveModule(
        DriveConstants.kBackRightDriveMotorPort,
        DriveConstants.kBackRightSteerMotorPort,
        DriveConstants.kBackRightCANcoderId,
        DriveConstants.kBackRightDriveMotorReversed,
        DriveConstants.kBackRightSteerMotorReversed,
        DriveConstants.kBackRightModuleCoderReversed,
        DriveConstants.kBackRightModuleCoderOffsetRad
    );

    // Gyro sensor (Pigeon2) tells us the robot's current heading (which way it's facing).
    private final Pigeon2 gyro = new Pigeon2(DriveConstants.kPigeonId);

    // Odometry estimates where the robot is on the field (x, y, and heading).
    // It uses:
    // - gyro heading
    // - each module's distance traveled + steering angle
    private SwerveDriveOdometry odometry;

    // Constructor: runs once when the robot code starts up.
    public SwerveSubsystem() {
        // Start with heading = 0 so "field oriented" has a clean reference.
        zeroHeading();

        // Create odometry with:
        // - the kinematics object (wheel geometry)
        // - initial heading
        // - initial module positions (distance + angle)
        odometry = new SwerveDriveOdometry(
            DriveConstants.kDriveKinematics,
            getRotation2d(),
            getModulePositions()
        );
    }

    // Sets the gyro yaw to zero (robot now believes it is facing "0 degrees").
    public void zeroHeading() {
        gyro.setYaw(0);
    }

    // Returns heading in degrees.
    // IEEEremainder keeps it in a nice -180..180-ish range instead of growing forever.
    public double getHeadingDeg() {
        return Math.IEEEremainder(gyro.getYaw().getValueAsDouble(), 360);
    }

    // WPILib often prefers Rotation2d instead of raw degrees/radians.
    // Rotation2d is just an "angle object" that can convert units for us.
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeadingDeg());
    }

    // Current estimated robot position on the field (meters + heading).
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    // Collect the 4 module positions into an array for odometry updates.
    // Each module position includes:
    // - distance traveled (meters)
    // - current module angle
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };
    }

    // periodic() runs automatically every robot loop (~20ms).
    // This is where subsystems update sensors, odometry, and dashboards.
    @Override
    public void periodic() {
        // Update odometry with current heading + module distances
        odometry.update(getRotation2d(), getModulePositions());

        // Helpful telemetry for debugging while driving
        SmartDashboard.putNumber("Heading (deg)", getHeadingDeg());
        SmartDashboard.putString("Pose", getPose().toString());
    }

    // Stops all modules (good for safety, end of commands, disabled, etc.)
    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    // Main "drive" method used by teleop commands.
    // Input: ChassisSpeeds (robot motion request: vx, vy, omega)
    // Output: Each module gets a SwerveModuleState (wheel speed + wheel angle)
    public void drive(ChassisSpeeds speeds) {
        // Convert the desired robot motion into 4 wheel commands
        SwerveModuleState[] states =
            DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

        // If any wheel speed is above the max, scale all speeds down together.
        // This keeps the requested movement direction the same, just slower.
        SwerveDriveKinematics.desaturateWheelSpeeds(
            states,
            DriveConstants.kPhysicalMaxSpeedMetersPerSecond
        );

        // Send each module its target state
        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
    }

    // Same idea as drive(...), but the caller already has the module states.
    // Commonly used in autonomous where a trajectory follower outputs module states.
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        // Clamp speeds to max allowed
        SwerveDriveKinematics.desaturateWheelSpeeds(
            desiredStates,
            DriveConstants.kPhysicalMaxSpeedMetersPerSecond
        );

        // Apply commands to modules
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    // Resets odometry to a known position (useful at start of auto).
    // Example: "we are at (0,0) facing forward" or wherever you place the robot.
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
    }
}