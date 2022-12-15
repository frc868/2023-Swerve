package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.techhounds.houndutil.houndlog.LogGroup;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.enums.LogType;
import com.techhounds.houndutil.houndlog.loggers.Logger;
import com.techhounds.houndutil.houndlog.loggers.SendableLogger;
import com.techhounds.houndutil.houndlog.loggers.SingleItemLogger;

import frc.robot.Constants;
import frc.robot.utils.SwerveModule;

/**
 * The drivetrain subsystem, containing four swerve modules and odometry-related
 * information.
 * 
 * @author dr
 */
public class Drivetrain extends SubsystemBase {
    /** The front left swerve module when looking at the bot from behind. */
    private SwerveModule frontLeft = new SwerveModule("Drivetrain/Front Left Module",
            Constants.Drivetrain.CANIDs.FrontLeft.DRIVE_MOTOR,
            Constants.Drivetrain.CANIDs.FrontLeft.TURN_MOTOR,
            Constants.Drivetrain.CANIDs.FrontLeft.TURN_ENCODER,
            false, true, false,
            Constants.Drivetrain.Offsets.FRONT_LEFT);

    /** The front right swerve module when looking at the bot from behind. */
    private SwerveModule frontRight = new SwerveModule("Drivetrain/Front Right Module",
            Constants.Drivetrain.CANIDs.FrontRight.DRIVE_MOTOR,
            Constants.Drivetrain.CANIDs.FrontRight.TURN_MOTOR,
            Constants.Drivetrain.CANIDs.FrontRight.TURN_ENCODER,
            false, true, false,
            Constants.Drivetrain.Offsets.FRONT_RIGHT);

    /** The back left swerve module when looking at the bot from behind. */
    private SwerveModule backLeft = new SwerveModule("Drivetrain/Back Left Module",
            Constants.Drivetrain.CANIDs.BackLeft.DRIVE_MOTOR,
            Constants.Drivetrain.CANIDs.BackLeft.TURN_MOTOR,
            Constants.Drivetrain.CANIDs.BackLeft.TURN_ENCODER,
            false, true, false,
            Constants.Drivetrain.Offsets.BACK_LEFT);

    /** The back right swerve module when looking at the bot from behind. */
    private SwerveModule backRight = new SwerveModule("Drivetrain/Back Right Module",
            Constants.Drivetrain.CANIDs.BackRight.DRIVE_MOTOR,
            Constants.Drivetrain.CANIDs.BackRight.TURN_MOTOR,
            Constants.Drivetrain.CANIDs.BackRight.TURN_ENCODER,
            false, true, false,
            Constants.Drivetrain.Offsets.BACK_RIGHT);

    /** The NavX, connected via MXP to the RoboRIO. */
    private Pigeon2 pigeon = new Pigeon2(0);

    /** Calculates odometry (robot's position) throughout the match. */
    private SwerveDriveOdometry odometry = new SwerveDriveOdometry(Constants.Drivetrain.Geometry.KINEMATICS,
            getGyroRotation2d());

    /** Field that the robot's position can be drawn on and send via NT. */
    private Field2d field = new Field2d();

    /** An enum describing the two types of drive modes. */
    public enum DriveMode {
        ROBOT_RELATIVE,
        FIELD_ORIENTED
    }

    /** The mode of driving, either robot relative or field relative. */
    private DriveMode driveMode = DriveMode.FIELD_ORIENTED;

    private double turnRegister = 0;

    /** Initializes the drivetrain. */
    public Drivetrain() {
        resetGyroAngle();
        LoggingManager.getInstance().addGroup("Drivetrain", new LogGroup(
                new Logger[] {
                        new SingleItemLogger<Double>(LogType.NUMBER, "Gyro Angle", this::getGyroAngle),
                        new SendableLogger("field", field),
                }));
    }

    /**
     * Runs every 20ms. Do not run anything but odometry updating and debug code
     * here.
     */
    @Override
    public void periodic() {
        odometry.update(
                getGyroRotation2d(),
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState());
        field.setRobotPose(odometry.getPoseMeters());
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(pose, getGyroRotation2d());
    }

    public DriveMode getDriveMode() {
        return driveMode;
    }

    public void setDriveMode(DriveMode driveMode) {
        this.driveMode = driveMode;
    }

    /**
     * Drives the drivetrain in a specified direction.
     * 
     * When driving the robot in a field-relative mode, positive x speeds correspond
     * to moving down the field.
     *
     * 
     * @param xSpeed        the speed in the x direction in m/s
     * @param ySpeed        the speed in the y direction in m/s
     * @param thetaSpeed    the rotational speed, in the counterclockwise direction,
     *                      and in rad/s (2pi is one rotation per second)
     * @param fieldRelative whether to control the robot relative to the field or to
     *                      the front of the bot
     */
    public void drive(double xSpeed, double ySpeed, double thetaSpeed, DriveMode driveMode) {
        ChassisSpeeds chassisSpeeds = null;
        switch (driveMode) {
            case ROBOT_RELATIVE:
                chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);
                break;
            case FIELD_ORIENTED:
                chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed,
                        thetaSpeed, getGyroRotation2d());
                break;
        }
        SwerveModuleState[] states = Constants.Drivetrain.Geometry.KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states,
                Constants.Drivetrain.Geometry.MAX_PHYSICAL_VELOCITY_METERS_PER_SECOND);
        setModuleStates(states);
    }

    /**
     * Sets the states of the swerve modules.
     * 
     * @param states an array of states (front left, front right, back left, back
     *               right)
     */
    public void setModuleStates(SwerveModuleState[] states) {
        frontLeft.setState(states[0]);
        frontRight.setState(states[1]);
        backLeft.setState(states[2]);
        backRight.setState(states[3]);
    }

    /**
     * Gets the angle of the gyro in degrees.
     * 
     * @return the angle of the gyro in degrees
     */
    public double getGyroAngle() {
        return pigeon.getYaw();
    }

    /**
     * Gets the angle of the gyro in degrees.
     * 
     * @return the angle of the gyro in degrees
     */
    public Rotation2d getGyroRotation2d() {
        return Rotation2d.fromDegrees(pigeon.getYaw());
    }

    /**
     * Reset gyro angle.
     */
    public void resetGyroAngle() {
        pigeon.setYaw(0);
    }

    /**
     * Stops the drivetrain.
     */
    public void stop() {
        drive(0, 0, 0, DriveMode.ROBOT_RELATIVE);
    }

    /**
     * Gets the current pose of the drivetrain.
     * 
     * @return the current pose of the drivetrain
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState()
        };
    }

    public void drawRobotOnField(Field2d field) {
        field.setRobotPose(getPose());

        // Draw a pose that is based on the robot pose, but shifted by the
        // translation of the module relative to robot center,
        // then rotated around its own center by the angle of the module.
        field.getObject("frontLeft").setPose(
                getPose().transformBy(
                        new Transform2d(Constants.Drivetrain.Geometry.SWERVE_MODULE_LOCATIONS[0],
                                getModuleStates()[0].angle)));
        field.getObject("frontRight").setPose(
                getPose().transformBy(
                        new Transform2d(Constants.Drivetrain.Geometry.SWERVE_MODULE_LOCATIONS[1],
                                getModuleStates()[1].angle)));
        field.getObject("backLeft").setPose(
                getPose().transformBy(
                        new Transform2d(Constants.Drivetrain.Geometry.SWERVE_MODULE_LOCATIONS[2],
                                getModuleStates()[2].angle)));
        field.getObject("backRight").setPose(
                getPose().transformBy(
                        new Transform2d(Constants.Drivetrain.Geometry.SWERVE_MODULE_LOCATIONS[3],
                                getModuleStates()[3].angle)));
    }

    /**
     * Adds 90 degrees CCW to the internal register for the turning PID controller.
     * Map this to a button input to turn 90 degrees CCW while still moving.
     */
    public void turnCCW90() {
        turnRegister += Math.PI / 2;
    }

    /**
     * Adds 90 degrees CW to the internal register for the turning PID controller.
     * Map this to a button input to turn 90 degrees CW while still moving.
     */
    public void turnCW90() {
        turnRegister -= Math.PI / 2;
    }

    /**
     * Get the amount that has been set in the drivetrain's turning register.
     * 
     * @return the angle
     */
    public double getTurnRegister() {
        return turnRegister;
    }

    /**
     * Resets the turning register. Do this after the bot has reached its turn
     * setpoint.
     */
    public void resetTurnRegister() {
        turnRegister = 0.0;
    }
}
