package frc.robot;

import java.util.Collections;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.TurnWheelsToAngle;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.DriveMode;

/**
 * The container for the robot. Contains subsystems, OI devices, and commands.
 */
public class RobotContainer {
    private final Drivetrain drivetrain = new Drivetrain();

    SendableChooser<Command> chooser = new SendableChooser<>();
    HashMap<String, Trajectory> trajectories = new HashMap<String, Trajectory>();

    /**
     * Constructs the robot container.
     */
    public RobotContainer() {
        LiveWindow.disableAllTelemetry(); // livewindow is basically deprecated. using houndlog instead.
        configureButtonBindings();
        configureAutonChooser();
        // loadTrajectories(); // no trajectories just yet.

    }

    private void configureAutonChooser() {
        // chooser.addOption("5 Ball",
        // SwerveTrajectoryBuilder.buildTrajectoryCommand(trajectories.get("5 Ball"),
        // drivetrain));
    }

    private void configureButtonBindings() {
        switch (Constants.CONTROLLER_TYPE) {
            case XboxController:
                XboxController driverController = new XboxController(Constants.OI.DRIVER_PORT);
                // Driver left joystick and right joystick, drive
                drivetrain.setDefaultCommand(
                        new TeleopDrive(
                                () -> driverController.getLeftX(),
                                () -> driverController.getLeftY(),
                                () -> driverController.getRightY(),
                                () -> false,
                                drivetrain));

            case FlightStick:
                Joystick joystick = new Joystick(0);

                drivetrain.setDefaultCommand(
                        new TeleopDrive(
                                () -> -joystick.getY() * 2.0, // because flight stick goes -0.5 to 0.5
                                () -> -joystick.getX() * 2.0,
                                () -> -joystick.getTwist() * 2.0,
                                () -> joystick.getRawButton(1),
                                drivetrain));

                new JoystickButton(joystick, 12).whenPressed(
                        new InstantCommand(drivetrain::resetGyroAngle).beforeStarting(new PrintCommand("resetting")));

                new JoystickButton(joystick, 4)
                        .whenPressed(new InstantCommand(() -> Constants.Teleop.PERCENT_LIMIT -= 0.05));
                new JoystickButton(joystick, 6)
                        .whenPressed(new InstantCommand(() -> Constants.Teleop.PERCENT_LIMIT += 0.05));

                new JoystickButton(joystick, 6).whileHeld(new RunCommand(() -> drivetrain.setModuleStates(
                        new SwerveModuleState[] {
                                new SwerveModuleState(0, new Rotation2d(0)),
                                new SwerveModuleState(0, new Rotation2d(0)),
                                new SwerveModuleState(0, new Rotation2d(0)),
                                new SwerveModuleState(0, new Rotation2d(0))
                        })));

                new JoystickButton(joystick, 3).whenPressed(new InstantCommand(drivetrain::turnCCW90));
                new JoystickButton(joystick, 4).whenPressed(new InstantCommand(drivetrain::turnCW90));

                new POVButton(joystick, 0).whenPressed(new TurnWheelsToAngle(0.0 * Math.PI / 4.0, drivetrain));
                new POVButton(joystick, 45).whenPressed(new TurnWheelsToAngle(1.0 * Math.PI / 4.0, drivetrain));
                new POVButton(joystick, 90).whenPressed(new TurnWheelsToAngle(2.0 * Math.PI / 4.0, drivetrain));
                new POVButton(joystick, 135).whenPressed(new TurnWheelsToAngle(3.0 * Math.PI / 4.0, drivetrain));
                new POVButton(joystick, 180).whenPressed(new TurnWheelsToAngle(4.0 * Math.PI / 4.0, drivetrain));
                new POVButton(joystick, 225).whenPressed(new TurnWheelsToAngle(5.0 * Math.PI / 4.0, drivetrain));
                new POVButton(joystick, 270).whenPressed(new TurnWheelsToAngle(6.0 * Math.PI / 4.0, drivetrain));
                new POVButton(joystick, 315).whenPressed(new TurnWheelsToAngle(7.0 * Math.PI / 4.0, drivetrain));

        }

    }

    public Command getAutonomousCommand() {
        PathPlannerTrajectory path = PathPlanner.loadPath("TestPath", 1, 3);
        // return new SequentialCommandGroup(
        // new InstantCommand(() -> {
        // // Reset odometry for the first path you run during auto
        // drivetrain.resetOdometry(path.getInitialPose());
        // }
        // ));

        ProfiledPIDController thetaController = new ProfiledPIDController(
                Constants.Drivetrain.PID.Drive.kP, 0, 0, new TrapezoidProfile.Constraints(
                        Constants.Drivetrain.Geometry.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                        Constants.Drivetrain.Geometry.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        return new PPSwerveControllerCommand(
                path,
                drivetrain::getPose, // Pose supplier
                Constants.Drivetrain.Geometry.KINEMATICS, // SwerveDriveKinematics
                new PIDController(Constants.Drivetrain.PID.Drive.kP, 0, 0),
                new PIDController(Constants.Drivetrain.PID.Drive.kP, 0, 0),
                thetaController, // Rotation controller. Tune these values for your robot. Leaving them 0 will
                                 // only use feedforwards.
                drivetrain::setModuleStates, // Module states consumer
                drivetrain // Requires this drive subsystem
        ).beforeStarting(() -> drivetrain.resetOdometry(path.getInitialPose())).andThen(drivetrain::stop);
    }

    // TrajectoryConfig config = new TrajectoryConfig(
    // Constants.Auton.MAX_VELOCITY,
    // Constants.Auton.MAX_ACCELERATION)
    // // Add kinematics to ensure max speed is actually obeyed
    // .setKinematics(Constants.Drivetrain.Geometry.KINEMATICS);

    // // An example trajectory to follow. All units in meters.
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    // // Start at the origin facing the +X direction
    // new Pose2d(0, 0, new Rotation2d(0)),
    // // Pass through these two interior waypoints, making an 's' curve path
    // List.of(new Translation2d(0.5, -0.5)),
    // // End 3 meters straight ahead of where we started, facing forward
    // new Pose2d(1, 0, new Rotation2d(0)),
    // config);

    // var thetaController = new ProfiledPIDController(
    // Constants.Drivetrain.PID.Drive.kP, 0, 0, new TrapezoidProfile.Constraints(
    // Constants.Drivetrain.Geometry.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
    // Constants.Drivetrain.Geometry.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED));
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // SwerveControllerCommand swerveControllerCommand = new
    // SwerveControllerCommand(
    // exampleTrajectory,
    // drivetrain::getPose, // Functional interface to feed supplier
    // Constants.Drivetrain.Geometry.KINEMATICS,
    // // Position controllers
    // new PIDController(Constants.Drivetrain.PID.Drive.kP, 0, 0),
    // new PIDController(Constants.Drivetrain.PID.Drive.kP, 0, 0),
    // thetaController,
    // drivetrain::setModuleStates,
    // drivetrain);

    // // Reset odometry to the starting pose of the trajectory.
    // drivetrain.resetOdometry(exampleTrajectory.getInitialPose());

    // // Run path following command, then stop at the end.
    // return swerveControllerCommand.andThen(() -> drivetrain.drive(0, 0, 0,
    // DriveMode.ROBOT_RELATIVE));
}
