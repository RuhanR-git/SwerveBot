package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase
{
    private final SwerveModule frontLeft = new SwerveModule
    (
        DriveConstants.kFrontLeftDriveMotorPort,
        DriveConstants.kFrontLeftSteerMotorPort,
        DriveConstants.kFrontLeftCANcoderId,
        DriveConstants.kFrontLeftModuleCoderReversed,
        DriveConstants.kFrontLeftModuleCoderOffsetRad
    );

    private final SwerveModule frontRight = new SwerveModule
    (
        DriveConstants.kFrontRightDriveMotorPort,
        DriveConstants.kFrontRightSteerMotorPort,
        DriveConstants.kFrontRightCANcoderId,
        DriveConstants.kFrontRightModuleCoderReversed,
        DriveConstants.kFrontRightModuleCoderOffsetRad
    );

    private final SwerveModule backLeft = new SwerveModule
    (
        DriveConstants.kBackLeftDriveMotorPort,
        DriveConstants.kBackLeftSteerMotorPort,
        DriveConstants.kBackLeftCANcoderId,
        DriveConstants.kBackLeftModuleCoderReversed,
        DriveConstants.kBackLeftModuleCoderOffsetRad
    );

    private final SwerveModule backRight = new SwerveModule
    (
        DriveConstants.kBackRightDriveMotorPort,
        DriveConstants.kBackRightSteerMotorPort,
        DriveConstants.kBackRightCANcoderId,
        DriveConstants.kBackRightModuleCoderReversed,
        DriveConstants.kBackRightModuleCoderOffsetRad
    );

   private final Pigeon2 gyro = new Pigeon2(DriveConstants.kPigeonId);

    public SwerveModuleState[] getModuleStates() 
    {
        return new SwerveModuleState[] 
        {
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        };
    }

    public Pose2d getPose() 
    {
        return m_odometry.getPoseMeters();
    }

    public SwerveModulePosition[] getModulePositions() 
    {
        return new SwerveModulePosition[] 
        {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };
    }

   private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics,
        getRotation2d(),
        getModulePositions()
    );

    public SwerveSubsystem() 
    {
        new Thread(() -> 
        {
            try 
            {
                Thread.sleep(1000);
                zeroHeading();
            } 
            catch (Exception e) 
            {
            }
        }).start();
    }

    public void zeroHeading() 
    {
        gyro.reset();
    }

    public double getHeading()
    {
        // Phoenix 6: yaw signal is the cleanest source
        double yawDeg = gyro.getYaw().getValueAsDouble();  // typically degrees

        return Math.IEEEremainder(yawDeg, 360);
    }

    public Rotation2d getRotation2d() 
    {
        return Rotation2d.fromDegrees(getHeading());
    }

    @Override
    public void periodic()
    {
        // Update pose estimate every loop
        m_odometry.update(getRotation2d(), getModulePositions());

        SmartDashboard.putNumber("Robot Heading (deg)", getHeading());
        SmartDashboard.putString("Robot Pose", getPose().toString());
    }


    public void stopModules() 
    {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void drive(ChassisSpeeds speeds) 
    {
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

    public SwerveModuleState[] getModulePositions(SwerveModuleState[] states) 
    {
        return states;
    }


    public void resetOdometry(Pose2d pose) 
    {
        // Note: getModulePositions() should return a SwerveModulePosition[]
        m_odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
    }
}