package frc.robot.utils;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.houndutil.houndlog.LogGroup;
import frc.houndutil.houndlog.LogProfileBuilder;
import frc.houndutil.houndlog.LoggingManager;
import frc.houndutil.houndlog.enums.LogLevel;
import frc.houndutil.houndlog.enums.LogType;
import frc.houndutil.houndlog.loggers.Logger;
import frc.houndutil.houndlog.loggers.SingleItemLogger;
import frc.houndutil.houndlog.loggers.DeviceLogger;
import frc.robot.Constants;

public class SwerveModule {
    /** The motor used for driving. */
    private CANSparkMax driveMotor;
    /** The motor used for turning. */
    private CANSparkMax turnMotor;

    /** The encoder on the motor used for driving. */
    private RelativeEncoder driveEncoder;
    /** The CANCoder used to tell the angle of the wheel. */
    private CANCoder turnCanCoder;

    /** The PID controller that corrects the drive motor's velocity. */
    private PIDController drivePIDController = new PIDController(Constants.Drivetrain.PID.Drive.kP,
            Constants.Drivetrain.PID.Drive.kI, Constants.Drivetrain.PID.Drive.kD);

    /** The PID controller that controls the turning motor's position. */
    private ProfiledPIDController turnPIDController = new ProfiledPIDController(
            Constants.Drivetrain.PID.Turn.kP,
            Constants.Drivetrain.PID.Turn.kI, Constants.Drivetrain.PID.Turn.kD,
            new TrapezoidProfile.Constraints(Constants.Drivetrain.Geometry.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                    Constants.Drivetrain.Geometry.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED));

    private PIDController turnPIDControllerSimple = new PIDController(Constants.Drivetrain.PID.Turn.kP,
            Constants.Drivetrain.PID.Turn.kI, Constants.Drivetrain.PID.Turn.kD);

    /** The feedforward controller that controls the drive motor's velocity. */
    private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(Constants.Drivetrain.PID.Drive.kS,
            Constants.Drivetrain.PID.Drive.kV);

    /**
     * Initalizes a SwerveModule.
     * 
     * @param name                 the name of the module (used for logging)
     * @param driveMotorChannel    the CAN ID of the drive motor
     * @param turnMotorChannel     the CAN ID of the turning motor
     * @param canCoderChannel      the CAN ID of the CANCoder
     * @param driveMotorInverted   if the drive motor is inverted
     * @param turnMotorInverted    if the turn motor is inverted
     * @param turnCanCoderInverted if the turn encoder is inverted
     */
    public SwerveModule(
            String name,
            int driveMotorChannel,
            int turnMotorChannel,
            int canCoderChannel,
            boolean driveMotorInverted,
            boolean turnMotorInverted,
            boolean turnCanCoderInverted) {

        driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
        driveMotor.setInverted(driveMotorInverted);
        driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        turnMotor = new CANSparkMax(turnMotorChannel, MotorType.kBrushless);
        turnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(2 * Math.PI * Constants.Drivetrain.Geometry.WHEEL_RADIUS_METERS); // in
                                                                                                                   // meters
        driveEncoder
                .setVelocityConversionFactor((2 * Math.PI * Constants.Drivetrain.Geometry.WHEEL_RADIUS_METERS) / 60.0);

        turnCanCoder = new CANCoder(canCoderChannel);

        // There is an issue with absolute position vs position in CANCoders, namely
        // that the abs pos is sent a lot less frequently than the normal pos (every
        // 100ms vs every 10ms). According to this post:
        // https://www.chiefdelphi.com/t/official-sds-mk3-mk4-code/397109/99, setting
        // the CANCoder to "Boot to Absolute" will fix this.
        turnCanCoder.setPositionToAbsolute();
        turnCanCoder.configSensorDirection(turnCanCoderInverted);
        turnCanCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        turnCanCoder.configFeedbackCoefficient(2 * Math.PI / 4096.0, "rad", SensorTimeBase.PerSecond); // radians/sec

        turnPIDController.enableContinuousInput(-Math.PI, Math.PI);

        LoggingManager.getInstance().addGroup(name, new LogGroup(
                new Logger[] {
                        new DeviceLogger<CANSparkMax>(driveMotor, "Drive Motor",
                                LogProfileBuilder.buildCANSparkMaxLogItems(driveMotor)),
                        new DeviceLogger<CANSparkMax>(turnMotor, "Turning Motor",
                                LogProfileBuilder.buildCANSparkMaxLogItems(turnMotor)),
                        new DeviceLogger<CANCoder>(turnCanCoder, "CANCoder",
                                LogProfileBuilder.buildCANCoderLogItems(turnCanCoder)),
                        new SingleItemLogger<Double>(LogType.NUMBER, "Wheel Angle", this::getAngle, LogLevel.MAIN),
                        new SingleItemLogger<Double>(LogType.NUMBER, "CANCoder Position", turnCanCoder::getPosition,
                                LogLevel.MAIN)

                }));
    }

    /**
     * Gets the state of the swerve module.
     * 
     * @return the state of the swerve module
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(turnCanCoder.getPosition()));
    }

    /**
     * Sets the PID controller setpoints to the desired state.
     * 
     * @param state the desired state of the swerve module
     */
    public void setState(SwerveModuleState state) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state,
                new Rotation2d(turnCanCoder.getPosition()));
        double driveOutput = drivePIDController.calculate(driveEncoder.getVelocity(),
                optimizedState.speedMetersPerSecond)
                + driveFeedforward.calculate(optimizedState.speedMetersPerSecond);
        double turnOutput = turnPIDController.calculate(turnCanCoder.getPosition(), optimizedState.angle.getRadians());

        driveMotor.set(driveOutput);
        turnMotor.set(turnOutput);
    }

    /**
     * Sets the PID controller setpoints to the desired state.
     * 
     * @param state the desired state of the swerve module
     */
    public void setStateSimple(SwerveModuleState state) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state,
                new Rotation2d(turnCanCoder.getPosition()));
        driveMotor.set(optimizedState.speedMetersPerSecond
                / Constants.Drivetrain.Geometry.MAX_PHYSICAL_VELOCITY_METERS_PER_SECOND);
        turnMotor.set(turnPIDControllerSimple.calculate(turnCanCoder.getPosition(),
                optimizedState.angle.getRadians()));
    }

    /**
     * Gets the position of the drive encoder.
     * 
     * @return the position of the drive encoder.
     */
    public double getDriveEncoderPosition() {
        return driveEncoder.getPosition();
    }

    /**
     * Reset the position of the encoder on the drive motor.
     */
    public void resetDriveEncoder() {
        driveEncoder.setPosition(0);
    }

    /**
     * Gets the current angle of the wheel.
     * 
     * @return the current angle of the wheel, in degrees, [0, 360) CCW.
     */
    public double getAngle() {
        return Math.toDegrees(turnCanCoder.getPosition());
    }
}