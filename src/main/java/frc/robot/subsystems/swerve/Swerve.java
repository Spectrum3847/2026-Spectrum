// Based on
// https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/SwerveWithPathPlanner/src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java
package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.rebuilt.Field;
import frc.rebuilt.FieldHelpers;
import frc.rebuilt.RobotBumpSim;
import frc.robot.Robot;
import frc.spectrumLib.swerve.MapleSimSwerveDrivetrain;
import frc.spectrumLib.telemetry.Telemetry;
import frc.spectrumLib.util.Util;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
public class Swerve extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {

    // ── State machine ──────────────────────────────────────────────────────────────────
    public enum WantedState {
        TELEOP_DRIVE,
        PILOT_AIM_AT_TARGET,
        CENTER_ROTATION_CHANGE_LAUNCHING,
        X_BRAKE,
        IDLE
    }

    public enum SystemState {
        TELEOP_DRIVE,
        PILOT_AIM_AT_TARGET,
        CENTER_ROTATION_CHANGE_LAUNCHING,
        X_BRAKE,
        IDLE
    }

    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLE;

    public static final double TRANSLATION_ERROR_MARGIN_METERS = Units.inchesToMeters(1.0);
    public static final double DRIVE_TO_POINT_STATIC_FRICTION_CONSTANT = 0.02;
    private static final double SKEW_COMPENSATION_SCALAR = -0.03;

    private final Translation2d TURRET_PIVOT_POINT = new Translation2d(0, 0);

    @Getter public final Pigeon2 pigeon = getPigeon2();

    // Cache the signal objects once - don't call pigeon.getPitch() every loop,
    // that re-allocates a request each time. Store these as fields and refresh them.
    private final StatusSignal<Angle> pitchSignal = pigeon.getPitch();
    private final StatusSignal<Angle> rollSignal = pigeon.getRoll();

    @Getter @Setter private double teleopVelocityCoefficient = 1.0;
    @Getter @Setter private double teleopRotationVelocityCoefficient = 1.0;

    @Getter private final SwerveConfig config;
    private Notifier simNotifier = null;

    private Alert pigeonAlert = new Alert("Pigeon IMU Disconnected", Alert.AlertType.kError);
    /**
     * Returns the pitch.
     *
     * @return the pitch
     */
    public Rotation2d getPitch() {
        return Rotation2d.fromDegrees(pitchSignal.refresh().getValue().in(Degrees));
    }
    /**
     * Returns the roll.
     *
     * @return the roll
     */
    public Rotation2d getRoll() {
        return Rotation2d.fromDegrees(rollSignal.refresh().getValue().in(Degrees));
    }

    private final SwerveRequest.ApplyRobotSpeeds AutoRequest =
            new SwerveRequest.ApplyRobotSpeeds()
                    .withDriveRequestType(DriveRequestType.Velocity)
                    .withSteerRequestType(SteerRequestType.Position)
                    .withDesaturateWheelSpeeds(true);

    private static final SwerveRequest.ApplyFieldSpeeds FIELD_CENTRIC_DRIVE =
            new SwerveRequest.ApplyFieldSpeeds()
                    .withDriveRequestType(DriveRequestType.Velocity)
                    .withSteerRequestType(SteerRequestType.Position);

    private final SwerveRequest.FieldCentricFacingAngle DRIVE_AT_ANGLE_REQUEST =
            new SwerveRequest.FieldCentricFacingAngle()
                    .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                    .withSteerRequestType(SwerveModule.SteerRequestType.Position);

    private final SwerveRequest.SwerveDriveBrake X_BRAKE = new SwerveRequest.SwerveDriveBrake();

    private final SwerveRequest.Idle IDLE_REQUEST = new SwerveRequest.Idle();

    /** Publishes raw CANcoder data for the tools/swerve-align web app. */
    private final SwerveAlignment alignment;

    /**
     * Constructs a new Swerve drive subsystem.
     *
     * @param config The configuration object containing drivetrain constants and module
     *     configurations.
     */
    public Swerve(SwerveConfig config) {
        super(
                TalonFX::new,
                TalonFX::new,
                CANcoder::new,
                config.getDrivetrainConstants(),
                250.0,
                MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(config.getModules()));

        this.config = config;

        if (Utils.isSimulation()) {
            startSimThread();
        }

        configurePathPlanner();

        // Configure heading PID on the shared drive-at-angle request
        DRIVE_AT_ANGLE_REQUEST.HeadingController =
                new PhoenixPIDController(
                        config.getKPRotationController(),
                        config.getKIRotationController(),
                        config.getKDRotationController());
        DRIVE_AT_ANGLE_REQUEST.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        DRIVE_AT_ANGLE_REQUEST
                .withDeadband(
                        config.getLinearSpeedAt12Volts().baseUnitMagnitude()
                                * config.getAimDeadband())
                .withRotationalDeadband(
                        config.getAngularSpeedAt12Volts().baseUnitMagnitude()
                                * config.getAimDeadband())
                .withMaxAbsRotationalRate(config.getAngularSpeedAt12Volts());

        this.register();

        optimizeBusUtilization();
        // Must come after optimizeBusUtilization(), which silences the CANcoder signals it wants.
        alignment = new SwerveAlignment(getModules(), config);
        registerTelemetry(this::log);

        Telemetry.print(getName() + " Subsystem Initialized");
    }

    // --------------------------------------------------------------------------------
    // Periodic and Setup Methods
    // --------------------------------------------------------------------------------
    /**
     * Minimum spacing between swerve state publishes, in seconds.
     *
     * <p>CTRE runs this callback on the odometry thread, not the main loop: in the 2026-09-05 17:10
     * log {@code Swerve/State/Pose} alone landed 197 records a second, the single largest producer
     * in a log that overran DogLog's queue and dropped data. Odometry still integrates at full
     * rate; only the telemetry is thinned, to a little above the 50 Hz main loop.
     */
    private static final double STATE_LOG_PERIOD_SECONDS = 0.02;

    private double lastStateLogSeconds = 0;

    /** Log. */
    protected void log(SwerveDriveState state) {
        double now = Timer.getFPGATimestamp();
        if (now - lastStateLogSeconds < STATE_LOG_PERIOD_SECONDS) {
            return;
        }
        lastStateLogSeconds = now;

        Telemetry.log("Swerve/State/Pose", state.Pose);
        Telemetry.log("Swerve/State/TargetStates", state.ModuleTargets);
        Telemetry.log("Swerve/State/MeasuredStates", state.ModuleStates);
        Telemetry.log("Swerve/State/MeasuredSpeeds", state.Speeds);
    }
    /** Logs the battery usage. */
    protected void logBatteryUsage() {
        // Each sum walks all four modules and refreshes their signals; compute each once.
        double steerSupplyCurrent = getSteerMotorSupplyCurrents();
        double driveSupplyCurrent = getDriveMotorSupplyCurrents();
        double driveStatorCurrent = getDriveMotorStatorCurrents();
        double steerStatorCurrent = getSteerMotorStatorCurrents();
        Robot.getBatteryLogger().reportCurrentUsage("Mechanisms/SwerveSteer", steerSupplyCurrent);
        Robot.getBatteryLogger().reportCurrentUsage("Mechanisms/SwerveDrive", driveSupplyCurrent);

        Telemetry.log("Swerve/Currents/DriveStatorCurrent", driveStatorCurrent);
        Telemetry.log("Swerve/Currents/SteerStatorCurrent", steerStatorCurrent);
        Telemetry.log("Swerve/Currents/DriveSupplyCurrent", driveSupplyCurrent);
        Telemetry.log("Swerve/Currents/SteerSupplyCurrent", steerSupplyCurrent);
    }
    /**
     * Returns the sum of the drive motor stator currents.
     *
     * @return the sum of the drive motor stator currents across all modules
     */
    protected double getDriveMotorStatorCurrents() {
        return Arrays.stream(getModules())
                .mapToDouble(module -> module.getDriveMotor().getStatorCurrent().getValueAsDouble())
                .sum();
    }
    /**
     * Returns the sum of the steer motor stator currents.
     *
     * @return the sum of the steer motor stator currents across all modules
     */
    protected double getSteerMotorStatorCurrents() {
        return Arrays.stream(getModules())
                .mapToDouble(module -> module.getSteerMotor().getStatorCurrent().getValueAsDouble())
                .sum();
    }
    /**
     * Returns the sum of the drive motor supply currents.
     *
     * @return the sum of the drive motor supply currents across all modules
     */
    protected double getDriveMotorSupplyCurrents() {
        return Arrays.stream(getModules())
                .mapToDouble(module -> module.getDriveMotor().getSupplyCurrent().getValueAsDouble())
                .sum();
    }
    /**
     * Returns the sum of the steer motor supply currents.
     *
     * @return the sum of the steer motor supply currents across all modules
     */
    protected double getSteerMotorSupplyCurrents() {
        return Arrays.stream(getModules())
                .mapToDouble(module -> module.getSteerMotor().getSupplyCurrent().getValueAsDouble())
                .sum();
    }

    /**
     * This method is called periodically and is used to update the pilot's perspective. It ensures
     * that the swerve drive system is aligned correctly based on the pilot's view.
     */
    @Override
    public void periodic() {
        systemState = handleStateTransition();
        applyStates();

        Telemetry.log("Swerve/WantedState", wantedState.toString());
        Telemetry.log("Swerve/SystemState", systemState.toString());
        Telemetry.log("Swerve/CurrentCommand", getCurrentCommandName());
        Telemetry.log("Swerve/TeleopVelocityCoefficient", getTeleopVelocityCoefficient());
        Telemetry.log(
                "Swerve/TeleopRotationVelocityCoefficient", getTeleopRotationVelocityCoefficient());
        logBatteryUsage();
        alignment.log();

        checkPigeonConnection();

        if (Utils.isSimulation()) {
            Telemetry.log("Sim/SimPose", getRobotPose());
            if (robotBumpSim != null) {
                Pose2d simPose =
                        mapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose();
                ChassisSpeeds robotRelSpeeds =
                        mapleSimSwerveDrivetrain.mapleSimDrive
                                .getDriveTrainSimulatedChassisSpeedsRobotRelative();
                ChassisSpeeds fieldRelSpeeds =
                        ChassisSpeeds.fromRobotRelativeSpeeds(
                                robotRelSpeeds, simPose.getRotation());
                // subticks=5 -> dt = 20ms/5 = 4ms sub-steps (matches MapleSim's 5ms period closely)
                simRobotPose3d = robotBumpSim.update(simPose, fieldRelSpeeds, 5);
                if (robotBumpSim.isOnRamp()) {
                    mapleSimSwerveDrivetrain.mapleSimDrive.setSimulationWorldPose(
                            robotBumpSim.getSimWorldPose(simPose));
                }
                Telemetry.log("Sim/RobotPose3d", simRobotPose3d);
            }
        }
    }

    // -----------------------------------------------------------------------
    // Subsystem Setup
    // -----------------------------------------------------------------------
    /**
     * Returns the current command name.
     *
     * @return the current command name
     */
    protected String getCurrentCommandName() {
        Command currentCommand = this.getCurrentCommand();
        if (currentCommand != null) {
            return currentCommand.getName();
        }

        return "none";
    }
    /** Handles the state transition. */
    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case TELEOP_DRIVE -> SystemState.TELEOP_DRIVE;
            case PILOT_AIM_AT_TARGET -> SystemState.PILOT_AIM_AT_TARGET;
            case CENTER_ROTATION_CHANGE_LAUNCHING -> SystemState.CENTER_ROTATION_CHANGE_LAUNCHING;
            case X_BRAKE -> SystemState.X_BRAKE;
            case IDLE -> SystemState.IDLE;
            default -> SystemState.IDLE;
        };
    }
    /** Applies the states. */
    private void applyStates() {
        switch (systemState) {
            default:
            case IDLE:
                setControl(IDLE_REQUEST);
                break;
            case TELEOP_DRIVE:
                setControl(FIELD_CENTRIC_DRIVE.withSpeeds(calculateSpeedsBasedOnJoystickInputs()));
                break;
            case CENTER_ROTATION_CHANGE_LAUNCHING:
                setControl(
                        FIELD_CENTRIC_DRIVE
                                .withSpeeds(calculateSpeedsBasedOnJoystickInputs())
                                .withCenterOfRotation(TURRET_PIVOT_POINT));
                break;
            case X_BRAKE:
                setControl(X_BRAKE);
                break;
        }
    }
    /** Calculates the speeds based on joystick inputs. */
    private ChassisSpeeds calculateSpeedsBasedOnJoystickInputs() {
        if (DriverStation.getAlliance().isEmpty()) {
            return new ChassisSpeeds(0, 0, 0);
        }

        double xMagnitude = Robot.getPilot().getDriveFwdPositive();
        double yMagnitude = Robot.getPilot().getDriveLeftPositive();
        double angularMagnitude = Robot.getPilot().getDriveCCWPositive();

        double xVelocity =
                (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                                        == DriverStation.Alliance.Blue
                                ? xMagnitude
                                : -xMagnitude)
                        * teleopVelocityCoefficient;
        double yVelocity =
                (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                                        == DriverStation.Alliance.Blue
                                ? yMagnitude
                                : -yMagnitude)
                        * teleopVelocityCoefficient;
        double angularVelocity = angularMagnitude * teleopRotationVelocityCoefficient;

        Rotation2d skewCompensationFactor =
                Rotation2d.fromRadians(
                        getCurrentRobotChassisSpeeds().omegaRadiansPerSecond
                                * SKEW_COMPENSATION_SCALAR);

        return ChassisSpeeds.fromRobotRelativeSpeeds(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        new ChassisSpeeds(xVelocity, yVelocity, angularVelocity),
                        getRobotPose().getRotation()),
                getRobotPose().getRotation().plus(skewCompensationFactor));
    }

    // --------------------------------------------------------------------------------
    // Pose Methods
    // --------------------------------------------------------------------------------

    /**
     * The function `getRobotPose` returns the robot's pose after checking and updating it.
     *
     * @return The `getRobotPose` method is returning the robot's current pose after calling the
     *     `seedCheckedPose` method with the current pose as an argument.
     */
    public Pose2d getRobotPose() {
        // Simulates collision by with field obstacles and boundaries
        if (this.mapleSimSwerveDrivetrain != null) {
            return mapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose();
        }
        return getState().Pose;
    }

    /**
     * Checks the connection status of the Pigeon IMU. If it is not connected, an alert will show up
     * in Elastic
     */
    private void checkPigeonConnection() {
        if (getPigeon() == null || !getPigeon().isConnected()) {
            pigeonAlert.set(true);
        } else {
            pigeonAlert.set(false);
        }
    }

    /**
     * Get the robot's pose at a specific timestamp using interpolation
     *
     * @param timestampSeconds The timestamp to sample at
     * @return The interpolated pose, or current pose if timestamp not in buffer
     */
    public Pose2d getPoseAtTimestamp(double timestampSeconds) {
        Optional<Pose2d> sampled = super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));

        return sampled.orElse(getRobotPose());
    }
    /** Resets the pose. */
    @Override
    public void resetPose(Pose2d pose) {
        if (this.mapleSimSwerveDrivetrain != null) {
            mapleSimSwerveDrivetrain.mapleSimDrive.setSimulationWorldPose(pose);
            Timer.delay(0.05); // Wait for simulation to update
        }
        super.resetPose(pose);
    }

    // --------------------------------------------------------------------------------
    // Zone Triggers
    // --------------------------------------------------------------------------------
    /** In xzone. */
    public Trigger inXzone(double minXmeter, double maxXmeter) {
        return new Trigger(
                () -> Util.inRange(() -> getRobotPose().getX(), () -> minXmeter, () -> maxXmeter));
    }
    /** In yzone. */
    public Trigger inYzone(double minYmeter, double maxYmeter) {
        return new Trigger(
                () -> Util.inRange(() -> getRobotPose().getY(), () -> minYmeter, () -> maxYmeter));
    }

    /**
     * This method is used to check if the robot is in the X zone of the field flips the values if
     * Red Alliance
     *
     * @param minXmeter the minimum X coordinate in meters
     * @param maxXmeter the maximum X coordinate in meters
     * @return the Trigger
     */
    public Trigger inXzoneAlliance(double minXmeter, double maxXmeter) {
        return new Trigger(
                () ->
                        Util.inRange(
                                FieldHelpers.flipXifRed(getRobotPose().getX()),
                                minXmeter,
                                maxXmeter));
    }

    /**
     * This method is used to check if the robot is in the Y zone of the field flips the values if
     * Red Alliance
     *
     * @param minYmeter the minimum Y coordinate in meters
     * @param maxYmeter the maximum Y coordinate in meters
     * @return the Trigger
     */
    public Trigger inYzoneAlliance(double minYmeter, double maxYmeter) {
        return new Trigger(
                () ->
                        Util.inRange(
                                FieldHelpers.flipYifRed(getRobotPose().getY()),
                                minYmeter,
                                maxYmeter));
    }

    private static final double FIELD_LENGTH_METERS = Field.fieldLength;
    private static final double FIELD_WIDTH_METERS = Field.fieldWidth;
    private static final double NEUTRAL_DEPTH_METERS = Units.inchesToMeters(283.0);
    private static final double NEUTRAL_LENGTH_METERS = Field.fieldWidth;
    private static final double ENEMY_ALLIANCE_DEPTH_METERS = Units.inchesToMeters(180.0);

    private static final Rectangle2d NEUTRAL_ZONE =
            new Rectangle2d(
                    new Translation2d(
                            FIELD_LENGTH_METERS / 2.0 - NEUTRAL_DEPTH_METERS / 2.0,
                            FIELD_WIDTH_METERS / 2.0 - NEUTRAL_LENGTH_METERS / 2.0),
                    new Translation2d(
                            FIELD_LENGTH_METERS / 2.0 + NEUTRAL_DEPTH_METERS / 2.0,
                            FIELD_WIDTH_METERS / 2.0 + NEUTRAL_LENGTH_METERS / 2.0));

    private static final Rectangle2d ENEMY_ALLIANCE_ZONE =
            new Rectangle2d(
                    new Translation2d(FIELD_LENGTH_METERS - ENEMY_ALLIANCE_DEPTH_METERS, 0),
                    new Translation2d(FIELD_LENGTH_METERS, FIELD_WIDTH_METERS));

    /** Returns {@code true} when the robot is inside the neutral zone. Allocation-free. */
    public boolean isInNeutralZone() {
        return NEUTRAL_ZONE.contains(getRobotPose().getTranslation());
    }

    /**
     * Returns {@code true} when the robot is inside the opposing alliance's zone (pose X is flipped
     * for red so the same rectangle works for both alliances).
     */
    public boolean isInEnemyAllianceZone() {
        Pose2d pose = getRobotPose();
        return ENEMY_ALLIANCE_ZONE.contains(
                new Translation2d(FieldHelpers.flipXifRed(pose.getX()), pose.getY()));
    }
    /** In neutral zone. */
    public Trigger inNeutralZone() {
        return new Trigger(this::isInNeutralZone);
    }
    /** In enemy alliance zone. */
    public Trigger inEnemyAllianceZone() {
        return new Trigger(this::isInEnemyAllianceZone);
    }
    /** In field right. */
    public Trigger inFieldRight() {
        final double fieldWidthMeters = Units.feetToMeters(27.0); // full field width (Y)
        final double halfWidth = fieldWidthMeters / 2.0;

        return new Trigger(() -> getRobotPose().getY() < halfWidth);
    }
    /** In field left. */
    public Trigger inFieldLeft() {
        final double fieldWidthMeters = Units.feetToMeters(27.0); // full field width (Y)
        final double halfWidth = fieldWidthMeters / 2.0;

        return new Trigger(() -> getRobotPose().getY() >= halfWidth);
    }

    // --------------------------------------------------------------------------------
    // Speed Checks
    // --------------------------------------------------------------------------------
    /**
     * Returns {@code true} if the robot is moving faster than the threshold.
     *
     * @param thresholdSpeed the speed threshold in meters per second
     * @return {@code true} if the current linear speed exceeds the threshold
     */
    public boolean isGoingTooFast(double thresholdSpeed) {
        ChassisSpeeds speeds = getCurrentRobotChassisSpeeds();
        double linearSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        return linearSpeed > thresholdSpeed;
    }
    /**
     * Returns a trigger that activates when the robot exceeds the speed threshold.
     *
     * @param thresholdSpeed the speed threshold in meters per second
     * @return a trigger active while {@code isGoingTooFast(thresholdSpeed)} returns {@code true}
     */
    public Trigger overSpeedTrigger(double thresholdSpeed) {
        return new Trigger(() -> isGoingTooFast(thresholdSpeed));
    }
    /**
     * Returns the current robot chassis speeds.
     *
     * @return the current robot chassis speeds
     */
    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return getKinematics().toChassisSpeeds(getState().ModuleStates);
    }

    // --------------------------------------------------------------------------------
    // Reorientation Methods
    // --------------------------------------------------------------------------------
    /** Reorient. */
    protected void reorient(double angleDegrees) {
        resetPose(
                new Pose2d(
                        getRobotPose().getX(),
                        getRobotPose().getY(),
                        Rotation2d.fromDegrees(angleDegrees)));
    }
    /** Reorient pilot angle. */
    protected Command reorientPilotAngle(double angleDegrees) {
        return runOnce(
                () -> {
                    double output = FieldHelpers.flipAngleIfRed(angleDegrees);
                    reorient(output);
                });
    }
    /**
     * Returns the nearest cardinal heading.
     *
     * @return the nearest cardinal angle in degrees (0, 90, 180, or 270)
     */
    protected double getClosestCardinal() {
        double heading = getRotation().getRadians();
        if (heading > -Math.PI / 4 && heading <= Math.PI / 4) {
            return 0;
        } else if (heading > Math.PI / 4 && heading <= 3 * Math.PI / 4) {
            return 90;
        } else if (heading > 3 * Math.PI / 4 || heading <= -3 * Math.PI / 4) {
            return 180;
        } else {
            return 270;
        }
    }
    /** Cardinal reorient. */
    protected Command cardinalReorient() {
        return runOnce(
                () -> {
                    double angleDegrees = getClosestCardinal();
                    reorient(angleDegrees);
                });
    }

    /**
     * Reorients the robot front away from the driver station. Angles are blue-origin and get
     * flipped on red.
     *
     * @return the reorient command
     */
    public Command reorientForward() {
        return reorientPilotAngle(0).withName("Swerve.reorientForward");
    }

    /**
     * Reorients the robot front to the driver's left.
     *
     * @return the reorient command
     */
    public Command reorientLeft() {
        return reorientPilotAngle(90).withName("Swerve.reorientLeft");
    }

    /**
     * Reorients the robot front back toward the driver station.
     *
     * @return the reorient command
     */
    public Command reorientBack() {
        return reorientPilotAngle(180).withName("Swerve.reorientBack");
    }

    /**
     * Reorients the robot front to the driver's right.
     *
     * @return the reorient command
     */
    public Command reorientRight() {
        return reorientPilotAngle(270).withName("Swerve.reorientRight");
    }
    /**
     * Returns {@code true} if the normal heading is closer to the target than the flipped heading.
     *
     * @param angleDegrees the target angle in degrees
     * @return {@code true} if the front heading is closer to the target than the flipped heading
     */
    public boolean frontClosestToAngle(double angleDegrees) {
        double heading = getRotation().getDegrees();
        double flippedHeading;
        if (heading > 0) {
            flippedHeading = heading - 180;
        } else {
            flippedHeading = heading + 180;
        }
        double frontDifference = getRotationDifference(heading, angleDegrees);
        double flippedDifference = getRotationDifference(flippedHeading, angleDegrees);

        return frontDifference < flippedDifference;
    }

    // Helper method to calculate the shortest angle difference
    /**
     * Returns the shortest absolute difference between two angles.
     *
     * @param angle1 the first angle in degrees
     * @param angle2 the second angle in degrees
     * @return the shortest difference between the angles, in the range 0-180 degrees
     */
    public double getRotationDifference(double angle1, double angle2) {
        double diff = Math.abs(angle1 - angle2) % 360;
        return diff > 180 ? 360 - diff : diff;
    }

    // --------------------------------------------------------------------------------
    // Rotation Controller
    // --------------------------------------------------------------------------------
    /**
     * Returns the rotation.
     *
     * @return the rotation
     */
    Rotation2d getRotation() {
        return getRobotPose().getRotation();
    }
    /**
     * Returns the rotation radians.
     *
     * @return the rotation radians
     */
    double getRotationRadians() {
        return getRobotPose().getRotation().getRadians();
    }

    // --------------------------------------------------------------------------------
    // Request Methods
    // --------------------------------------------------------------------------------

    // Used to set a control request to the swerve module, ignores disable so commands are
    // continuous.
    /** Applies the request. */
    Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get())).ignoringDisable(true);
    }

    // ── Public state setters ───────────────────────────────────────────────────────────
    /**
     * Sets the wanted state.
     *
     * @param state the wanted state
     */
    public void setWantedState(WantedState state) {
        this.wantedState = state;
    }
    /**
     * Returns {@code true} if the at desired rotation condition is met.
     *
     * @return {@code true} if the at desired rotation condition is met
     */
    public boolean isAtDesiredRotation() {
        return isAtDesiredRotation(Units.degreesToRadians(10.0));
    }
    /**
     * Returns {@code true} if the heading controller position error is within tolerance.
     *
     * @param toleranceRadians the allowed heading error in radians
     * @return {@code true} if the heading controller position error is below the tolerance
     */
    public boolean isAtDesiredRotation(double toleranceRadians) {
        return Math.abs(DRIVE_AT_ANGLE_REQUEST.HeadingController.getPositionError())
                < toleranceRadians;
    }

    // --------------------------------------------------------------------------------
    // Path Planner Configuration
    // --------------------------------------------------------------------------------
    /** Configures the path planner. */
    private void configurePathPlanner() {
        // Seed robot to in front of blue hub (Paths will change this starting position)
        resetPose(
                new Pose2d(
                        Field.getBlueHubCenter().getX() - 2,
                        Field.getBlueHubCenter().getY(),
                        Rotation2d.fromDegrees(0)));

        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    this::getRobotPose, // Supplier of current robot pose
                    this::resetPose, // Consumer for seeding pose against auto
                    this::getCurrentRobotChassisSpeeds, // Supplier of current robot speeds
                    // Consumer of ChassisSpeeds and feedforwards to drive the robot
                    (speeds, feedforwards) -> {
                        setControl(
                                AutoRequest.withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))
                                        .withWheelForceFeedforwardsX(
                                                feedforwards.robotRelativeForcesX())
                                        .withWheelForceFeedforwardsY(
                                                feedforwards.robotRelativeForcesY()));
                    },
                    new PPHolonomicDriveController(
                            // PID constants for translation
                            new PIDConstants(4, 0, 0),
                            // PID constants for rotation
                            new PIDConstants(4, 0, 0)),
                    config,
                    // Assume the path needs to be flipped for Red vs Blue, this is normally the
                    // case
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                    this // Subsystem for requirements
                    );
        } catch (Exception ex) {
            DriverStation.reportError(
                    "Failed to load PathPlanner config and configure AutoBuilder",
                    ex.getStackTrace());
        }
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    // Simulated drivetrain used for robot bump simulation.
    @Getter private MapleSimSwerveDrivetrain mapleSimSwerveDrivetrain = null;

    @Getter private RobotBumpSim robotBumpSim = null;
    @Getter private Pose3d simRobotPose3d = Pose3d.kZero;

    /** Starts the sim thread. */
    @SuppressWarnings("unchecked")
    private void startSimThread() {
        mapleSimSwerveDrivetrain =
                new MapleSimSwerveDrivetrain(
                        Seconds.of(config.getSimLoopPeriod()),
                        Pounds.of(115), // robot weight
                        Inches.of(30), // bumper length
                        Inches.of(30), // bumper width
                        DCMotor.getKrakenX60Foc(1), // drive motor type
                        DCMotor.getKrakenX60Foc(1), // steer motor type
                        1.2, // wheel COF
                        getModuleLocations(),
                        getPigeon2(),
                        getModules(),
                        config.getFrontLeft(),
                        config.getFrontRight(),
                        config.getBackLeft(),
                        config.getBackRight());
        robotBumpSim = new RobotBumpSim(getModuleLocations());

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        simNotifier = new Notifier(mapleSimSwerveDrivetrain::update);
        simNotifier.startPeriodic(config.getSimLoopPeriod());
    }
}
