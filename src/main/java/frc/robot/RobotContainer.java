package frc.robot; // Package for the robot code

import java.util.List; // Java List class

import edu.wpi.first.wpilibj.Joystick; // WPILib Joystick class

import edu.wpi.first.math.controller.PIDController; // WPILib PID controller class
import edu.wpi.first.math.controller.ProfiledPIDController; // WPILib Profiled PID controller class

import edu.wpi.first.math.geometry.Pose2d; // WPILib robot pose class
import edu.wpi.first.math.geometry.Rotation2d; // WPILib rotation (angle) class
import edu.wpi.first.math.geometry.Translation2d; // WPILib 2D translation (x,y) class

import edu.wpi.first.math.trajectory.Trajectory; // WPILib trajectory class
import edu.wpi.first.math.trajectory.TrajectoryConfig; // WPILib trajectory configuration class
import edu.wpi.first.math.trajectory.TrajectoryGenerator; // WPILib trajectory generator class

import edu.wpi.first.wpilibj2.command.Command; // WPILib Command base class
import edu.wpi.first.wpilibj2.command.Commands; //  WPILib Commands helper class
import edu.wpi.first.wpilibj2.command.InstantCommand; // WPILib InstantCommand class
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup; // WPILib SequentialCommandGroup class
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand; // WPILib SwerveControllerCommand class
import edu.wpi.first.wpilibj2.command.button.JoystickButton; // WPILib JoystickButton class

import frc.robot.Constants.AutoConstants; // Auto constants
import frc.robot.Constants.DriveConstants; // Drive constants
import frc.robot.Constants.OIConstants; // Operator Interface constants

import frc.robot.commands.SwerveJoyStickCommand; // Swerve Joystick command for teleop
import frc.robot.subsystems.SwerveSubsystem; // Swerve drivetrain subsystem

// RobotContainer is the "wiring hub" of the robot code.
// It is responsible for:
// - Creating subsystems (hardware owners)
// - Setting default commands (what runs during teleop)
// - Binding buttons to actions
// - Providing the autonomous command to run in auto
public class RobotContainer {

    // Subsystem instance (drivetrain)
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    // Driver input device (Joystick can also represent an Xbox controller in WPILib)
    private final Joystick driverJoystick = new Joystick(OIConstants.kDriverJoystickPort);

    public RobotContainer() {
        // Default teleop drive command:
        // The scheduler runs this command automatically whenever nothing else is using the drivetrain.
        //
        // We pass in "Suppliers" (lambdas) so the command can read joystick values LIVE every loop.
        // Python analogy: passing functions like lambda: joystick_value
        swerveSubsystem.setDefaultCommand(new SwerveJoyStickCommand(
            swerveSubsystem,

            // Forward/back axis (negated so pushing forward = positive forward)
            () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),

            // Left/right strafe axis
            () ->  driverJoystick.getRawAxis(OIConstants.kDriverXAxis),

            // Rotation axis (turning)
            () ->  driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),

            // Field oriented toggle/hold:
            // This Supplier returns true/false depending on the button state.
            () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)
        ));

        // Set up button -> action mappings
        configureButtonBindings();
    }

    // Button bindings are kept in one method so RobotContainer stays organized.
    private void configureButtonBindings() {
        // Zero heading on button 2:
        // JoystickButton(...) creates a "trigger" for a physical button.
        // onTrue(...) means: run the command once when the button is pressed.
        new JoystickButton(driverJoystick, 2)
            .onTrue(Commands.runOnce(() -> swerveSubsystem.zeroHeading(), swerveSubsystem));
    }

    // This is called by Robot.java when autonomous starts.
    // Whatever Command we return here will be scheduled and run during auto.
    public Command getAutonomousCommand() {

        // 1) Trajectory config:
        // Sets max speed + max acceleration for trajectory generation.
        // Also tells WPILib the drivetrain geometry (kinematics) so it can plan correctly.
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared
        ).setKinematics(DriveConstants.kDriveKinematics);

        // 2) Generate a test trajectory:
        // Start pose -> interior waypoints -> end pose
        //
        // Pose2d(x, y, rotation) uses meters for x/y and radians/degrees via Rotation2d.
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(1, 0), new Translation2d(1, -1)),
            new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
            trajectoryConfig
        );

        // 3) Controllers:
        // These controllers are used by SwerveControllerCommand to correct errors while following the path.
        // - xController corrects x position error
        // - yController corrects y position error
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);

        // thetaController corrects heading error (rotation).
        // "Profiled" means it follows constraints so it doesn't try to spin infinitely fast.
        ProfiledPIDController thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints
        );

        // Heading wraps around (180° is the same direction as -180°),
        // so we make the controller treat the angle as circular.
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4) Follow command:
        // This command will:
        // - look at the current robot pose (from odometry)
        // - compare it to the trajectory target pose
        // - compute the needed robot motion
        // - convert that into module states
        // - send those module states to the drivetrain
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            trajectory,

            // How the command gets the robot's current position
            swerveSubsystem::getPose,

            // Drivetrain geometry
            DriveConstants.kDriveKinematics,

            // Position controllers
            xController,
            yController,

            // Rotation controller
            thetaController,

            // How the command outputs to the drivetrain:
            // It produces SwerveModuleState[] and we pass them to the subsystem method.
            swerveSubsystem::setModuleStates,
            swerveSubsystem
        );

        // 5) Wrap with init + stop:
        // We build a sequence of commands that run in order:
        // - zero heading
        // - reset odometry to match the start of the path
        // - follow the path
        // - stop the modules at the end
        return new SequentialCommandGroup(
            new InstantCommand(() -> swerveSubsystem.zeroHeading()),
            new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
            swerveControllerCommand,
            new InstantCommand(() -> swerveSubsystem.stopModules())
        );
    }
}