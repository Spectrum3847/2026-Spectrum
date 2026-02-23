package frc.robot.swerve;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.rebuilt.Field;
import frc.rebuilt.Zones;
import frc.robot.Robot;
import frc.robot.RobotStates;
import frc.robot.State;
import frc.robot.operator.Operator;
import frc.robot.pilot.Pilot;
import frc.spectrumLib.Telemetry;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.Set;
import java.util.function.DoubleSupplier;

public class SwerveStates {
    static Swerve swerve = Robot.getSwerve();
    static SwerveConfig config = Robot.getConfig().swerve;
    static Pilot pilot = Robot.getPilot();
    static Operator operator = Robot.getOperator();
    static Zones zones = new Zones();
    static Field field = new Field();

    /* Swerve Request Types */
    private static final SwerveRequest.FieldCentric fieldCentricDrive =
            new SwerveRequest.FieldCentric()
                    .withDeadband(
                            config.getSpeedAt12Volts().in(MetersPerSecond) * config.getDeadband())
                    .withRotationalDeadband(config.getMaxAngularRate() * config.getDeadband())
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private static final SwerveRequest.RobotCentric robotCentric =
            new SwerveRequest.RobotCentric()
                    .withDeadband(
                            config.getSpeedAt12Volts().in(MetersPerSecond) * config.getDeadband())
                    .withRotationalDeadband(config.getMaxAngularRate() * config.getDeadband())
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private static final SwerveRequest.SwerveDriveBrake swerveXBreak =
            new SwerveRequest.SwerveDriveBrake();

    private static final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngle =
            new SwerveRequest.FieldCentricFacingAngle()
                    .withDeadband(
                            config.getSpeedAt12Volts().in(MetersPerSecond) * config.getDeadband())
                    .withRotationalDeadband(config.getMaxAngularRate() * config.getDeadband())
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                    .withSteerRequestType(SteerRequestType.MotionMagicExpo)
                    .withMaxAbsRotationalRate(config.getMaxAngularRate())
                    .withHeadingPID(
                            config.getKPRotationController(),
                            config.getKIRotationController(),
                            config.getKDRotationController());

    public static Trigger robotInNeutralZone() {
        return swerve.inNeutralZone();
    }

    public static Trigger robotInEnemyZone() {
        return swerve.inEnemyAllianceZone();
    }

    static Command pilotSteerCommand =
            log(pilotDrive().withName("SwerveCommands.pilotSteer").ignoringDisable(true));

    protected static void setupDefaultCommand() {
        swerve.setDefaultCommand(pilotSteerCommand);
    }

    // define Triggers here
    private static final Trigger inSnakeDrive =
            new Trigger(() -> RobotStates.getAppliedState() == State.SNAKE_INTAKE);
    private static final Trigger inScoreOrFeed =
            new Trigger(
                    () ->
                            RobotStates.getAppliedState() == State.TURRET_WITHOUT_TRACK_WITH_LAUNCH
                                    || RobotStates.getAppliedState()
                                            == State.TURRET_FEED_WITH_LAUNCH
                                    || RobotStates.getAppliedState()
                                            == State.TURRET_TRACK_WITH_LAUNCH);

    protected static void setStates() {
        // Force back to manual steering when we steer
        pilot.steer.whileTrue(swerve.getDefaultCommand());

        pilot.fpv_LS.whileTrue(log(fpvDrive()));

        inSnakeDrive.whileTrue(log(snakeDrive()));
        inScoreOrFeed.whileTrue(log(tweakOut()));

        pilot.upReorient.onTrue(log(reorientForward()));
        pilot.leftReorient.onTrue(log(reorientLeft()));
        pilot.downReorient.onTrue(log(reorientBack()));
        pilot.rightReorient.onTrue(log(reorientRight()));
    }

    /* ------------------------- Pilot Commands --------------------------------------- */

    /** Drive the robot using left stick and control orientation using the right stick. */
    protected static Command pilotDrive() {
        return drive(
                        pilot::getDriveFwdPositive,
                        pilot::getDriveLeftPositive,
                        pilot::getDriveCCWPositive)
                .withName("Swerve.PilotDrive");
    }

    /** Drive the robot with its front bumper as the forward direction. */
    protected static Command fpvDrive() {
        return fpvDrive(
                        pilot::getDriveFwdPositive,
                        pilot::getDriveLeftPositive,
                        pilot::getDriveCCWPositive)
                .withName("Swerve.PilotFPVDrive");
    }

    /** Align the robot to the given x, y, and heading goals. */
    public static Command alignDrive(
            DoubleSupplier xGoalMeters, DoubleSupplier yGoalMeters, DoubleSupplier headingRadians) {
        if (Field.isRed()) {
            return resetXController()
                    .andThen(
                            resetYController(),
                            resetTurnController(),
                            drive(
                                    () -> -getAlignToX(xGoalMeters).getAsDouble(),
                                    () -> -getAlignToY(yGoalMeters).getAsDouble(),
                                    () -> getAlignHeading(headingRadians, true).getAsDouble()));
        }

        return resetXController()
                .andThen(
                        resetYController(),
                        resetTurnController(),
                        drive(
                                getAlignToX(xGoalMeters),
                                getAlignToY(yGoalMeters),
                                getAlignHeading(headingRadians, true)));
    }

    /** Drive the robot with the robot's orientation snapping to the closest cardinal direction. */
    protected static Command snapSteerDrive() {
        return drive(
                        pilot::getDriveFwdPositive,
                        pilot::getDriveLeftPositive,
                        pilot::chooseCardinalDirections)
                .withName("Swerve.PilotStickSteer");
    }

    /** Drive the robot with the front bumper trying to match the robot's motion. */
    protected static Command snakeDrive() {
        return aimDrive(
                        pilot::getDriveFwdPositive,
                        pilot::getDriveLeftPositive,
                        pilot::getPilotStickAngle)
                .withName("Swerve.SnakeDrive");
    }

    /**
     * Tweak the robot's orientation by a small angle back and forth.
     *
     * @return A command that repeatedly tweaks the robot's orientation.
     */
    protected static Command tweakOut() {
        return Commands.defer(
                        () -> {
                            final double base = swerve.getRotation().getRadians();
                            final double delta = Math.toRadians(15.0);

                            Command toMinus =
                                    aimDrive(
                                                    pilot::getDriveFwdPositive,
                                                    pilot::getDriveLeftPositive,
                                                    () -> base - delta)
                                            .withTimeout(0.5);

                            Command toPlus =
                                    aimDrive(
                                                    pilot::getDriveFwdPositive,
                                                    pilot::getDriveLeftPositive,
                                                    () -> base + delta)
                                            .withTimeout(0.5);

                            return Commands.repeatingSequence(toMinus, toPlus);
                        },
                        Set.of(swerve))
                .withName("Swerve.tweakOut");
    }

    /** Turn the swerve wheels to an X to prevent the robot from moving. */
    protected static Command xBrake() {
        return swerve.applyRequest(() -> swerveXBreak).withName("Swerve.Xbrake");
    }

    /**
     * Drive the robot with the front bumper trying to match a target angle. The target angle can be
     * specified in degrees.
     *
     * @param targetDegrees The target angle in degrees
     * @return A command that drives the robot to match the target angle while allowing translation
     *     control with the left stick
     */
    protected static Command pilotAimDrive(DoubleSupplier targetDegrees) {
        return aimDrive(pilot::getDriveFwdPositive, pilot::getDriveLeftPositive, targetDegrees)
                .withName("Swerve.PilotAimDrive");
    }

    private static DoubleSupplier getAlignToX(DoubleSupplier xGoalMeters) {
        return swerve.calculateXController(xGoalMeters);
    }

    private static DoubleSupplier getAlignToY(DoubleSupplier yGoalMeters) {
        return swerve.calculateYController(yGoalMeters);
    }

    private static DoubleSupplier getAlignHeading(DoubleSupplier headingRadians, boolean usehold) {
        return () -> swerve.calculateRotationController(headingRadians, usehold);
    }

    protected static Command headingLockDrive() {
        return headingLock(pilot::getDriveFwdPositive, pilot::getDriveLeftPositive)
                .withName("Swerve.PilotHeadingLockDrive");
    }

    protected static Command lockToClosest45degDrive() {
        return lockToClosest45deg(pilot::getDriveFwdPositive, pilot::getDriveLeftPositive)
                .withName("Swerve.PilotLockTo45degDrive");
    }

    protected static Command lockToClosestFieldAngleDrive() {
        return lockToClosestFieldAngle(pilot::getDriveFwdPositive, pilot::getDriveLeftPositive)
                .withName("Swerve.PilotLockToFieldAngleDrive");
    }

    // **********************   Helper Commands    ****************************

    protected static Command resetXController() {
        return swerve.runOnce(() -> swerve.resetXController()).withName("ResetXController");
    }

    protected static Command resetYController() {
        return swerve.runOnce(() -> swerve.resetYController()).withName("ResetYController");
    }

    protected static Command resetTurnController() {
        return swerve.runOnce(() -> swerve.resetRotationController())
                .withName("ResetTurnController");
    }

    protected static Command setTargetHeading(DoubleSupplier targetHeading) {
        return Commands.runOnce(() -> config.setTargetHeading(targetHeading.getAsDouble()))
                .withName("SetTargetHeading");
    }

    /**
     * Applies a field-centric drive request with the specified velocities (m/s) and rotational rate
     * (rad/s). The drive request will be processed by the swerve subsystem, which will handle the
     * necessary calculations to convert these inputs into individual wheel speeds and angles.
     *
     * @param fwdPositive Forward velocity in meters per second
     * @param leftPositive Leftward velocity in meters per second
     * @param ccwPositive Counter-clockwise rotational rate in radians per second
     * @return A command that applies the specified field-centric drive request to the swerve
     *     subsystem.
     */
    private static Command drive(
            DoubleSupplier fwdPositive, DoubleSupplier leftPositive, DoubleSupplier ccwPositive) {
        return swerve.applyRequest(
                        () ->
                                fieldCentricDrive
                                        .withVelocityX(fwdPositive.getAsDouble())
                                        .withVelocityY(leftPositive.getAsDouble())
                                        .withRotationalRate(ccwPositive.getAsDouble()))
                .withName("Swerve.drive");
    }

    /**
     * Applies a robot-centric drive request with the specified velocities (m/s) and rotational rate
     * (rad/s). The drive request will be processed by the swerve subsystem, which will handle the
     * necessary calculations to convert these inputs into individual wheel speeds and angles.
     *
     * @param fwdPositive Forward velocity in meters per second
     * @param leftPositive Leftward velocity in meters per second
     * @param ccwPositive Counter-clockwise rotational rate in radians per second
     * @return A command that applies the specified robot-centric drive request to the swerve
     *     subsystem.
     */
    private static Command fpvDrive(
            DoubleSupplier fwdPositive, DoubleSupplier leftPositive, DoubleSupplier ccwPositive) {
        return swerve.applyRequest(
                        () ->
                                robotCentric
                                        .withVelocityX(fwdPositive.getAsDouble())
                                        .withVelocityY(leftPositive.getAsDouble())
                                        .withRotationalRate(ccwPositive.getAsDouble()))
                .withName("Swerve.fpvDrive");
    }

    /**
     * Reset the turn controller and then run the field-centric drive command with a angle supplier.
     * This can be used for aiming at a goal or heading locking, etc.
     *
     * @param velocityX The forward velocity in meters per second
     * @param velocityY The leftward velocity in meters per second
     * @param targetRadians The target heading in radians that the robot should align to while
     *     driving
     * @return A command that resets the turn controller and then applies a field-centric drive
     *     request with the specified velocities and target heading.
     */
    protected static Command fpvAimDrive(
            DoubleSupplier velocityX, DoubleSupplier velocityY, DoubleSupplier targetRadians) {
        return resetTurnController()
                .andThen(fpvDrive(velocityX, velocityY, getAlignHeading(targetRadians, true)))
                .withName("Swerve.fpvAimDrive");
    }

    /**
     * Applies a field-centric drive request with the specified velocities (m/s) and a target
     * heading. The robot will attempt to maintain the target heading while driving.
     *
     * @param velocityX The forward velocity in meters per second
     * @param velocityY The leftward velocity in meters per second
     * @param targetRadians The target heading in radians that the robot should align to while
     *     driving
     * @return A command that applies a field-centric drive request with the specified velocities
     *     and target heading.
     */
    protected static Command aimDrive(
            DoubleSupplier velocityX, DoubleSupplier velocityY, DoubleSupplier targetRadians) {
        return swerve.applyRequest(
                        () ->
                                fieldCentricFacingAngle
                                        .withVelocityX(velocityX.getAsDouble())
                                        .withVelocityY(velocityY.getAsDouble())
                                        .withTargetDirection(
                                                new Rotation2d(targetRadians.getAsDouble())))
                .withName("Swerve.aimDrive");
    }

    /**
     * Reset the turn controller, set the target heading to the current heading(end that command
     * immediately), and then run the drive command with the Rotation controller. The rotation
     * controller will only engage if you are driving x or y.
     *
     * @param velocityX The forward velocity in meters per second
     * @param velocityY The leftward velocity in meters per second
     * @return A command that returns an aimDrive command that locks the robot's heading to its current heading while allowing translation control.
     */
    protected static Command headingLock(DoubleSupplier velocityX, DoubleSupplier velocityY) {
        return aimDrive(velocityX, velocityY, () -> swerve.getRotation().getRadians())
                .withName("Swerve.HeadingLock");
    }

    /**
     * Reset the turn controller, set the target heading to the closest 45 degree angle, and then
     * run the drive command with the Rotation controller. The rotation controller will only engage
     * if you are driving x or y.
     *
     * @param velocityX The forward velocity in meters per second
     * @param velocityY The leftward velocity in meters per second
     * 
     * @return A command that returns an aimDrive command that locks the robot's heading to the closest 45 degree angle while allowing translation control.
     */
    protected static Command lockToClosest45deg(
            DoubleSupplier velocityX, DoubleSupplier velocityY) {
        return aimDrive(velocityX, velocityY, () -> swerve.getClosest45())
                .withName("Swerve.LockTo45deg");
    }

    protected static Command lockToClosestFieldAngle(
            DoubleSupplier velocityX, DoubleSupplier velocityY) {
        return resetTurnController()
                .andThen(
                        setTargetHeading(() -> swerve.getClosestFieldAngle()),
                        drive(
                                velocityX,
                                velocityY,
                                rotateToHeadingWhenMoving(
                                        velocityX, velocityY, () -> swerve.getClosestFieldAngle())))
                .withName("Swerve.LockToFieldAngle");
    }

    private static DoubleSupplier rotateToHeadingWhenMoving(
            DoubleSupplier velocityX, DoubleSupplier velocityY, DoubleSupplier heading) {
        return () -> {
            if (Math.abs(velocityX.getAsDouble()) < 0.5
                    && Math.abs(velocityY.getAsDouble()) < 0.5) {
                return 0;
            } else {
                return getAlignHeading(heading, true).getAsDouble();
            }
        };
    }

    // --------------------------------------------------------------------------------
    // Swerve Characterization Routines
    // --------------------------------------------------------------------------------
    private static final double WHEEL_RADIUS_MAX_VELOCITY = 1; // Rad/Sec
    private static final double WHEEL_RADIUS_RAMP_RATE = 0.5; // Rad/Sec^2

    public static double[] getWheelRadiusCharacterizationPositions() {
        double[] positions = new double[4];
        double wheelRadiusGuess = config.getWheelRadius().in(Meters); // current config value

        for (int i = 0; i < 4; i++) {
            positions[i] =
                    swerve.getModule(i).getCachedPosition().distanceMeters / wheelRadiusGuess;
        }
        return positions;
    }

    /**
     * Measures the robot's wheel radius by spinning in a circle. (Method from AdvantageKit). <br>
     * </br> This command will ramp up the robot's rotation rate to a specified maximum while
     * recording the change in gyro angle and wheel positions. When the command is cancelled, it
     * will calculate and print the effective wheel radius based on the recorded data.
     */
    public static Command wheelRadiusCharacterization() {
        SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

        return Commands.parallel(
                // Drive control sequence
                Commands.sequence(
                        // Reset acceleration limiter
                        Commands.runOnce(
                                () -> {
                                    limiter.reset(0.0);
                                }),

                        // Turn in place, accelerating up to full speed
                        Commands.run(
                                () -> {
                                    double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                                    swerve.setControl(
                                            fieldCentricDrive
                                                    .withVelocityX(0)
                                                    .withVelocityY(0)
                                                    .withRotationalRate(speed));
                                },
                                swerve)),

                // Measurement sequence
                Commands.sequence(
                        // Wait for modules to fully orient before starting measurement
                        Commands.waitSeconds(1.0),

                        // Record starting measurement
                        Commands.runOnce(
                                () -> {
                                    state.positions = getWheelRadiusCharacterizationPositions();
                                    state.lastAngle = swerve.getRotation();
                                    state.gyroDelta = 0.0;
                                }),

                        // Update gyro delta
                        Commands.run(
                                        () -> {
                                            var rotation = swerve.getRotation();
                                            state.gyroDelta +=
                                                    Math.abs(
                                                            rotation.minus(state.lastAngle)
                                                                    .getRadians());
                                            state.lastAngle = rotation;
                                        })

                                // When cancelled, calculate and print results
                                .finallyDo(
                                        () -> {
                                            double[] positions =
                                                    getWheelRadiusCharacterizationPositions();
                                            double wheelDelta = 0.0;
                                            for (int i = 0; i < 4; i++) {
                                                wheelDelta +=
                                                        Math.abs(positions[i] - state.positions[i])
                                                                / 4.0;
                                            }
                                            double wheelRadius =
                                                    (state.gyroDelta
                                                                    * config
                                                                            .getDrivebaseRadiusMeters())
                                                            / wheelDelta;

                                            NumberFormat formatter = new DecimalFormat("#0.000");
                                            System.out.println(
                                                    "********** Wheel Radius Characterization Results **********");
                                            System.out.println(
                                                    "\tWheel Delta: "
                                                            + formatter.format(wheelDelta)
                                                            + " radians");
                                            System.out.println(
                                                    "\tGyro Delta: "
                                                            + formatter.format(state.gyroDelta)
                                                            + " radians");
                                            System.out.println(
                                                    "\tWheel Radius: "
                                                            + formatter.format(wheelRadius)
                                                            + " meters, "
                                                            + formatter.format(
                                                                    Units.metersToInches(
                                                                            wheelRadius))
                                                            + " inches");
                                        })));
    }

    private static class WheelRadiusCharacterizationState {
        double[] positions = new double[4];
        Rotation2d lastAngle = Rotation2d.kZero;
        double gyroDelta = 0.0;
    }

    // ------------------------------------------------------------------------
    // Reorient Commands
    // ------------------------------------------------------------------------
    protected static Command reorientForward() {
        return swerve.reorientPilotAngle(0).withName("Swerve.reorientForward");
    }

    protected static Command reorientLeft() {
        return swerve.reorientPilotAngle(90).withName("Swerve.reorientLeft");
    }

    protected static Command reorientBack() {
        return swerve.reorientPilotAngle(180).withName("Swerve.reorientBack");
    }

    protected static Command reorientRight() {
        return swerve.reorientPilotAngle(270).withName("Swerve.reorientRight");
    }

    protected static Command cardinalReorient() {
        return swerve.cardinalReorient().withName("Swerve.cardinalReorient");
    }

    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
