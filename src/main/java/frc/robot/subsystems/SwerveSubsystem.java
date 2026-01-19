package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
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

    private final Pigeon2 gyro = new Pigeon2(DriveConstants.kPigeonId);

    private SwerveDriveOdometry odometry;

    public SwerveSubsystem() {
        zeroHeading();
        odometry = new SwerveDriveOdometry(
            DriveConstants.kDriveKinematics,
            getRotation2d(),
            getModulePositions()
        );
    }

    public void zeroHeading() {
        gyro.setYaw(0);
    }

    public double getHeadingDeg() {
        return Math.IEEEremainder(gyro.getYaw().getValueAsDouble(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeadingDeg());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };
    }

    @Override
    public void periodic() {
        odometry.update(getRotation2d(), getModulePositions());
        SmartDashboard.putNumber("Heading (deg)", getHeadingDeg());
        SmartDashboard.putString("Pose", getPose().toString());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void drive(ChassisSpeeds speeds) {
        SwerveModuleState[] states =
            DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(
            states,
            DriveConstants.kPhysicalMaxSpeedMetersPerSecond
        );

        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
            desiredStates,
            DriveConstants.kPhysicalMaxSpeedMetersPerSecond
        );

        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
    }
}