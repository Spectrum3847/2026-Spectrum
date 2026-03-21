package frc.robot.swerve;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

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
import frc.rebuilt.ShotCalculator;
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
    // ------------------------------------------------------------------------
    // Dependencies / singletons
    // ------------------------------------------------------------------------
    private static final Swerve swerve = Robot.getSwerve();
    private static final SwerveConfig config = Robot.getConfig().swerve;
    private static final Pilot pilot = Robot.getPilot();
    private static final Operator operator = Robot.getOperator();

    // ------------------------------------------------------------------------
    // Requests (pre-configured CTRE swerve requests)
    // ------------------------------------------------------------------------
    private static final SwerveRequest.FieldCentric FIELD_CENTRIC_DRIVE =
            new SwerveRequest.FieldCentric()
                    .withDeadband(
                            config.getSpeedAt12Volts().in(MetersPerSecond) * config.getDeadband())
                    .withRotationalDeadband(config.getMaxAngularRate() * config.getDeadband())
                    .withDriveRequestType(DriveRequestType.Velocity);

    private static final SwerveRequest.RobotCentric ROBOT_CENTRIC_DRIVE =
            new SwerveRequest.RobotCentric()
                    .withDeadband(
                            config.getSpeedAt12Volts().in(MetersPerSecond) * config.getDeadband())
                    .withRotationalDeadband(config.getMaxAngularRate() * config.getDeadband())
                    .withDriveRequestType(DriveRequestType.Velocity);

    private static final SwerveRequest.SwerveDriveBrake X_BRAKE_REQUEST =
            new SwerveRequest.SwerveDriveBrake();

    private static final SwerveRequest.FieldCentricFacingAngle FIELD_CENTRIC_FACING_ANGLE =
            new SwerveRequest.FieldCentricFacingAngle()
                    .withDeadband(
                            config.getSpeedAt12Volts().in(MetersPerSecond) * config.getDeadband())
                    .withRotationalDeadband(config.getMaxAngularRate() * config.getDeadband())
                    .withDriveRequestType(DriveRequestType.Velocity)
                    .withSteerRequestType(SteerRequestType.MotionMagicExpo)
                    .withMaxAbsRotationalRate(config.getMaxAngularRate())
                    .withHeadingPID(
                            config.getKPRotationController(),
                            config.getKIRotationController(),
                            config.getKDRotationController());

    private static final SwerveRequest.RobotCentricFacingAngle ROBOT_CENTRIC_FACING_ANGLE =
            new SwerveRequest.RobotCentricFacingAngle()
                    .withDeadband(
                            config.getSpeedAt12Volts().in(MetersPerSecond) * config.getDeadband())
                    .withRotationalDeadband(config.getMaxAngularRate() * config.getDeadband())
                    .withDriveRequestType(DriveRequestType.Velocity)
                    .withSteerRequestType(SteerRequestType.MotionMagicExpo)
                    .withMaxAbsRotationalRate(config.getMaxAngularRate())
                    .withHeadingPID(
                            config.getKPRotationController(),
                            config.getKIRotationController(),
                            config.getKDRotationController());

    // ------------------------------------------------------------------------
    // Triggers
    // ------------------------------------------------------------------------
    @SuppressWarnings("unused")
    private static final Trigger snakeDrive =
            new Trigger(() -> RobotStates.getAppliedState() == State.SNAKE_INTAKE);

    private static final Trigger launching =
            new Trigger(() -> RobotStates.getAppliedState() == State.TURRET_TRACK_WITH_LAUNCH);

    private static final Trigger launchPreping =
            new Trigger(() -> RobotStates.getAppliedState() == State.TURRET_TRACK);

    public static Trigger robotInNeutralZone() {
        return swerve.inNeutralZone();
    }

    public static Trigger robotInEnemyZone() {
        return swerve.inEnemyAllianceZone();
    }

    // ------------------------------------------------------------------------
    // Default command
    // ------------------------------------------------------------------------
    private static final Command pilotSteerCommand =
            log(pilotDrive().withName("SwerveCommands.pilotSteer").ignoringDisable(true));

    protected static void setupDefaultCommand() {
        swerve.setDefaultCommand(pilotSteerCommand);
    }

    // ------------------------------------------------------------------------
    // Bindings / state setup
    // ------------------------------------------------------------------------
    protected static void setStates() {
        // Force back to manual steering when we steer
        pilot.steer.whileTrue(swerve.getDefaultCommand());
        operator.steer.whileTrue(swerve.getDefaultCommand());

        pilot.fpv_LS.whileTrue(log(fpvDrive()));

        launching.or(launchPreping).whileTrue(log(pilotAimAtTarget()));
        // launching.and(Robot.getPilot().fn).whileTrue(log(tweakOut()));
        // launching.and(Robot.getPilot().RB).whileTrue(log(xBrake()));

        pilot.upReorient.onTrue(log(reorientForward()));
        pilot.leftReorient.onTrue(log(reorientLeft()));
        pilot.downReorient.onTrue(log(reorientBack()));
        pilot.rightReorient.onTrue(log(reorientRight()));
    }

    // ------------------------------------------------------------------------
    // Pilot commands (public-facing)
    // ------------------------------------------------------------------------

    /**
     * Drive the robot using left stick and control orientation using the right stick.
     *
     * @return A command that drives the robot with translation control from the left stick and
     *     rotation control from the right stick
     */
    protected static Command pilotDrive() {
        return drive(
                        pilot::getDriveFwdPositive,
                        pilot::getDriveLeftPositive,
                        pilot::getDriveCCWPositive)
                .withName("Swerve.PilotDrive");
    }

    /**
     * Drive the robot with the front bumper trying to match the angle to the target.
     *
     * @return A command that drives the robot to match the angle to the target while allowing
     *     translation control with the left stick
     */
    protected static Command pilotAimAtTarget() {
        return aimDrive(
                        pilot::getDriveFwdPositive,
                        pilot::getDriveLeftPositive,
                        () ->
                                ShotCalculator.getInstance()
                                        .getParameters()
                                        .fieldAngle()
                                        .plus(Rotation2d.k180deg)
                                        .getRadians())
                .withName("Swerve.pilotAimAtTarget");
    }

    /**
     * Drive the robot with its front bumper as the forward direction.
     *
     * @return A command that drives the robot with the front bumper as the forward direction, using
     *     the left stick for translation and the right stick for rotation
     */
    protected static Command fpvDrive() {
        return fpvDrive(
                        pilot::getDriveFwdPositive,
                        pilot::getDriveLeftPositive,
                        pilot::getDriveCCWPositive)
                .withName("Swerve.PilotFPVDrive");
    }

    /**
     * Drive the robot with the robot's orientation snapping to the closest cardinal direction.
     *
     * @return A command that drives the robot with the robot's orientation snapping to the closest
     *     cardinal direction, using the left stick for translation
     */
    protected static Command snapSteerDrive() {
        return drive(
                        pilot::getDriveFwdPositive,
                        pilot::getDriveLeftPositive,
                        pilot::chooseCardinalDirections)
                .withName("Swerve.PilotStickSteer");
    }

    /**
     * Drive the robot with the front bumper trying to match the robot's motion direction.
     *
     * @return A command that drives the robot with translation control from the left stick, while
     *     also trying to match the robot's motion direction with the robot angle
     */
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
                            final double delta = Math.toRadians(10.0);

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
        return swerve.applyRequest(() -> X_BRAKE_REQUEST).withName("Swerve.Xbrake");
    }

    /**
     * Drive the robot with the front bumper trying to match a target angle.
     *
     * @param targetDegrees The target angle (expected to be in radians in current call sites)
     * @return A command that drives the robot to match the target angle while allowing translation
     *     control with the left stick
     */
    protected static Command pilotAimDrive(DoubleSupplier targetDegrees) {
        return aimDrive(pilot::getDriveFwdPositive, pilot::getDriveLeftPositive, targetDegrees)
                .withName("Swerve.PilotAimDrive");
    }

    /**
     * Align the robot to the given x, y, and heading goals.
     *
     * @param xGoalMeters The x goal in meters
     * @param yGoalMeters The y goal in meters
     * @param headingRadians The heading goal in radians
     * @return A command that aligns the robot to the specified x, y, and heading goals
     */
    public static Command alignDrive(
            DoubleSupplier xGoalMeters, DoubleSupplier yGoalMeters, DoubleSupplier headingRadians) {

        final DoubleSupplier x = getAlignToX(xGoalMeters);
        final DoubleSupplier y = getAlignToY(yGoalMeters);

        final boolean invertForRed = Field.isRed();

        return Commands.sequence(
                resetXController(),
                resetYController(),
                aimDrive(
                        () -> invertForRed ? -x.getAsDouble() : x.getAsDouble(),
                        () -> invertForRed ? -y.getAsDouble() : y.getAsDouble(),
                        headingRadians));
    }

    protected static Command headingLockDrive() {
        return headingLock(pilot::getDriveFwdPositive, pilot::getDriveLeftPositive)
                .withName("Swerve.PilotHeadingLockDrive");
    }

    protected static Command lockToClosest45Drive() {
        return lockToClosest45deg(pilot::getDriveFwdPositive, pilot::getDriveLeftPositive)
                .withName("Swerve.PilotLockTo45degDrive");
    }

    // ------------------------------------------------------------------------
    // Controller/heading helpers
    // ------------------------------------------------------------------------
    private static DoubleSupplier getAlignToX(DoubleSupplier xGoalMeters) {
        return swerve.calculateXController(xGoalMeters);
    }

    private static DoubleSupplier getAlignToY(DoubleSupplier yGoalMeters) {
        return swerve.calculateYController(yGoalMeters);
    }

    // ------------------------------------------------------------------------
    // Small helper commands (reset/set)
    // ------------------------------------------------------------------------
    protected static Command resetXController() {
        return swerve.runOnce(swerve::resetXController).withName("ResetXController");
    }

    protected static Command resetYController() {
        return swerve.runOnce(swerve::resetYController).withName("ResetYController");
    }

    protected static Command resetTurnController() {
        return swerve.runOnce(swerve::resetRotationController).withName("ResetTurnController");
    }

    protected static Command setTargetHeading(DoubleSupplier targetHeading) {
        return Commands.runOnce(() -> config.setTargetHeading(targetHeading.getAsDouble()))
                .withName("SetTargetHeading");
    }

    // ------------------------------------------------------------------------
    // Core drive primitives (compose these into higher-level behaviors)
    // ------------------------------------------------------------------------
    private static Command drive(
            DoubleSupplier fwdPositive, DoubleSupplier leftPositive, DoubleSupplier ccwPositive) {
        return swerve.applyRequest(
                        () ->
                                FIELD_CENTRIC_DRIVE
                                        .withVelocityX(fwdPositive.getAsDouble())
                                        .withVelocityY(leftPositive.getAsDouble())
                                        .withRotationalRate(ccwPositive.getAsDouble()))
                .withName("Swerve.drive");
    }

    private static Command fpvDrive(
            DoubleSupplier fwdPositive, DoubleSupplier leftPositive, DoubleSupplier ccwPositive) {
        return swerve.applyRequest(
                        () ->
                                ROBOT_CENTRIC_DRIVE
                                        .withVelocityX(fwdPositive.getAsDouble())
                                        .withVelocityY(leftPositive.getAsDouble())
                                        .withRotationalRate(ccwPositive.getAsDouble()))
                .withName("Swerve.fpvDrive");
    }

    protected static Command fpvAimDrive(
            DoubleSupplier velocityX, DoubleSupplier velocityY, DoubleSupplier targetRadians) {
        return swerve.applyRequest(
                        () ->
                                ROBOT_CENTRIC_FACING_ANGLE
                                        .withVelocityX(velocityX.getAsDouble())
                                        .withVelocityY(velocityY.getAsDouble())
                                        .withTargetDirection(
                                                new Rotation2d(targetRadians.getAsDouble())))
                .withName("Swerve.fpvAimDrive");
    }

    protected static Command aimDrive(
            DoubleSupplier velocityX, DoubleSupplier velocityY, DoubleSupplier targetRadians) {
        return swerve.applyRequest(
                        () ->
                                FIELD_CENTRIC_FACING_ANGLE
                                        .withVelocityX(velocityX.getAsDouble())
                                        .withVelocityY(velocityY.getAsDouble())
                                        .withTargetDirection(
                                                new Rotation2d(targetRadians.getAsDouble())))
                .withName("Swerve.aimDrive");
    }

    protected static Command headingLock(DoubleSupplier velocityX, DoubleSupplier velocityY) {
        return aimDrive(velocityX, velocityY, () -> swerve.getRotation().getRadians())
                .withName("Swerve.HeadingLock");
    }

    protected static Command lockToClosest45deg(
            DoubleSupplier velocityX, DoubleSupplier velocityY) {
        return aimDrive(velocityX, velocityY, swerve::getClosest45).withName("Swerve.LockTo45deg");
    }

    // ------------------------------------------------------------------------
    // Swerve characterization routines
    // ------------------------------------------------------------------------
    private static final double WHEEL_RADIUS_MAX_VELOCITY = 1; // rad/s
    private static final double WHEEL_RADIUS_RAMP_RATE = 0.5; // rad/s^2

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
     * Measures the robot's wheel radius by spinning in a circle. (Method from AdvantageKit).
     *
     * <p>This command ramps up the robot's rotation rate to a specified maximum while recording the
     * change in gyro angle and wheel positions. When the command is cancelled, it calculates and
     * prints the effective wheel radius based on the recorded data.
     */
    public static Command wheelRadiusCharacterization() {
        SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

        return Commands.parallel(
                // Drive control sequence
                Commands.sequence(
                        Commands.runOnce(() -> limiter.reset(0.0)),
                        Commands.run(
                                () -> {
                                    double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                                    swerve.setControl(
                                            FIELD_CENTRIC_DRIVE
                                                    .withVelocityX(0)
                                                    .withVelocityY(0)
                                                    .withRotationalRate(speed));
                                },
                                swerve)),

                // Measurement sequence
                Commands.sequence(
                        Commands.waitSeconds(1.0),
                        Commands.runOnce(
                                () -> {
                                    state.positions = getWheelRadiusCharacterizationPositions();
                                    state.lastAngle = swerve.getRotation();
                                    state.gyroDelta = 0.0;
                                }),
                        Commands.run(
                                        () -> {
                                            var rotation = swerve.getRotation();
                                            state.gyroDelta +=
                                                    Math.abs(
                                                            rotation.minus(state.lastAngle)
                                                                    .getRadians());
                                            state.lastAngle = rotation;
                                        })
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
                                            Telemetry.log(
                                                    "WheelRadiusCharacterization/WheelDelta",
                                                    formatter.format(wheelDelta) + " radians");
                                            Telemetry.log(
                                                    "WheelRadiusCharacterization/GyroDelta",
                                                    formatter.format(state.gyroDelta) + " radians");
                                            Telemetry.log(
                                                    "WheelRadiusCharacterization/WheelRadiusMeters",
                                                    formatter.format(wheelRadius) + " meters");
                                            Telemetry.log(
                                                    "WheelRadiusCharacterization/WheelRadiusInches",
                                                    formatter.format(
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
    // Reorient commands
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

    // ------------------------------------------------------------------------
    // Telemetry
    // ------------------------------------------------------------------------
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
