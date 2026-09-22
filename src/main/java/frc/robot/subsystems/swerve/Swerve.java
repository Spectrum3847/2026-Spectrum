// Based on
// https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/SwerveWithPathPlanner/src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java
package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
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
import frc.spectrumLib.framework.RobotLoop;
import frc.spectrumLib.hardware.CanConfigBudget;
import frc.spectrumLib.swerve.MapleSimSwerveDrivetrain;
import frc.spectrumLib.telemetry.Telemetry;
import java.util.Optional;
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
        X_BRAKE,
        IDLE
    }

    public enum SystemState {
        TELEOP_DRIVE,
        X_BRAKE,
        IDLE
    }

    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLE;

    private static final double SKEW_COMPENSATION_SCALAR = -0.03;

    @Getter public final Pigeon2 pigeon = getPigeon2();

    @Getter @Setter private double teleopVelocityCoefficient = 1.0;
    @Getter @Setter private double teleopRotationVelocityCoefficient = 1.0;

    @Getter private final SwerveConfig config;
    private Notifier simNotifier = null;

    private Alert pigeonAlert = new Alert("Pigeon IMU Disconnected", Alert.AlertType.kError);

    private final SwerveRequest.ApplyRobotSpeeds AutoRequest =
            new SwerveRequest.ApplyRobotSpeeds()
                    .withDriveRequestType(DriveRequestType.Velocity)
                    .withSteerRequestType(SteerRequestType.Position)
                    .withDesaturateWheelSpeeds(true);

    private static final SwerveRequest.ApplyFieldSpeeds FIELD_CENTRIC_DRIVE =
            new SwerveRequest.ApplyFieldSpeeds()
                    .withDriveRequestType(DriveRequestType.Velocity)
                    .withSteerRequestType(SteerRequestType.Position);

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

        this.register();

        // Eight motors' worth of per-signal config calls, and only an optimisation: on a dead bus
        // it is boot latency for nothing, so it is skipped once the CAN config budget is spent.
        if (!CanConfigBudget.exhausted()) {
            optimizeBusUtilization();
        }
        // Must come after optimizeBusUtilization(), which silences the CANcoder signals it wants.
        alignment = new SwerveAlignment(getModules(), config);

        var modules = getModules();
        moduleCurrentSignals = new BaseStatusSignal[modules.length * 4];
        moduleCurrentKeys = new String[modules.length * 4];
        moduleConnectedKeys = new String[modules.length * 2];
        for (int i = 0; i < modules.length; i++) {
            moduleCurrentSignals[4 * i] = modules[i].getDriveMotor().getStatorCurrent(false);
            moduleCurrentSignals[4 * i + 1] = modules[i].getDriveMotor().getSupplyCurrent(false);
            moduleCurrentSignals[4 * i + 2] = modules[i].getSteerMotor().getStatorCurrent(false);
            moduleCurrentSignals[4 * i + 3] = modules[i].getSteerMotor().getSupplyCurrent(false);

            // Built once so the 10 Hz tick does no string concatenation.
            String module =
                    i < SwerveAlignment.MODULE_NAMES.length
                            ? SwerveAlignment.MODULE_NAMES[i]
                            : "Module" + i;
            moduleCurrentKeys[4 * i] = CURRENTS_PREFIX + module + "/DriveStatorCurrent";
            moduleCurrentKeys[4 * i + 1] = CURRENTS_PREFIX + module + "/DriveSupplyCurrent";
            moduleCurrentKeys[4 * i + 2] = CURRENTS_PREFIX + module + "/SteerStatorCurrent";
            moduleCurrentKeys[4 * i + 3] = CURRENTS_PREFIX + module + "/SteerSupplyCurrent";
            moduleConnectedKeys[2 * i] = "Swerve/Modules/" + module + "/DriveConnected";
            moduleConnectedKeys[2 * i + 1] = "Swerve/Modules/" + module + "/SteerConnected";
        }

        Telemetry.print(getName() + " Subsystem Initialized");
    }

    // --------------------------------------------------------------------------------
    // Periodic and Setup Methods
    // --------------------------------------------------------------------------------
    // ── Per-loop drivetrain state ──────────────────────────────────────────────

    /** Snapshot of the drivetrain state for this loop; see {@link #loopState()}. */
    private SwerveDriveState loopState;

    /** Loop the snapshot was taken on, or -1 for none. */
    private long loopStateLoop = -1;

    /**
     * The drivetrain state, read once per loop.
     *
     * <p>{@code getState()} is a JNI call plus the odometry lock plus a copy, and the pose was
     * being pulled that way a dozen times a loop (vision, shot calculator, superstructure, turret,
     * Field2d, zone triggers). Every caller in a loop now sees the same snapshot, which is also the
     * right semantics: a shot solution and the turret aiming it should agree on where the robot is.
     * The snapshot is dropped whenever the pose is changed on purpose (vision fusion, resets), so a
     * correction made early in the loop is seen by everything after it.
     *
     * @return this loop's drivetrain state
     */
    protected SwerveDriveState loopState() {
        long loop = RobotLoop.count();
        if (loopState == null || loopStateLoop != loop) {
            loopState = getState().clone();
            loopStateLoop = loop;
        }
        return loopState;
    }

    /** Forgets this loop's state snapshot so the next read sees a pose change made this loop. */
    private void invalidateLoopState() {
        loopStateLoop = -1;
    }

    /**
     * Logs pose, module targets, module states and chassis speeds from the main loop.
     *
     * <p>This used to be CTRE's {@code registerTelemetry} callback, which runs on the odometry
     * thread while it holds the drivetrain state lock. Four struct serializations and a queue put
     * per call happened under that lock, and when the DogLog queue filled, the queue-full report
     * with its stack trace did too. In the 2026-09-05 Driver Station logs the dropped entries were
     * exactly these four keys, and {@code Swerve.periodic} showed up at 100-200 ms in overrun
     * traces, waiting on that lock from {@code setControl}. Logging the loop's own snapshot here
     * costs the odometry thread nothing and keeps odometry at 250 Hz.
     */
    private void logSwerveState() {
        SwerveDriveState state = loopState();
        Telemetry.log("Swerve/State/Pose", state.Pose);
        Telemetry.log("Swerve/State/TargetStates", state.ModuleTargets);
        Telemetry.log("Swerve/State/MeasuredStates", state.ModuleStates);
        Telemetry.log("Swerve/State/MeasuredSpeeds", state.Speeds);
    }

    // ── Currents ──────────────────────────────────────────────────────────────

    /** NetworkTables/DogLog key prefix for the drivetrain's current telemetry. */
    private static final String CURRENTS_PREFIX = "Swerve/Currents/";

    /** Drive and steer stator and supply current signals for every module, refreshed together. */
    private BaseStatusSignal[] moduleCurrentSignals = new BaseStatusSignal[0];

    /** Log key for each entry of {@link #moduleCurrentSignals}, in the same order. */
    private String[] moduleCurrentKeys = new String[0];

    /**
     * {@code Swerve/Modules/<name>/DriveConnected} and {@code .../SteerConnected}, two per module.
     *
     * <p>The only connection telemetry the swerve had was {@code Swerve/Align/.../Connected}, which
     * publishes while disabled. In the 2026-09-19 Chezy P8 match the CANivore bus died mid-teleop
     * and the eight drivetrain motors left no direct evidence at all; the failure had to be read
     * off their currents at 10 Hz. These ride the same 10 Hz refresh as the currents.
     */
    private String[] moduleConnectedKeys = new String[0];

    private double driveStatorCurrent;
    private double driveSupplyCurrent;
    private double steerStatorCurrent;
    private double steerSupplyCurrent;

    /**
     * Reports drive and steer supply current to the battery logger every loop and logs the four
     * current sums at 10 Hz.
     *
     * <p>The sixteen module current signals are refreshed in one Phoenix call on the 10 Hz tick and
     * held between ticks. They used to be refreshed one JNI call each, every loop, through four
     * streams; the battery logger's energy integral tolerates a 100 ms sample-and-hold.
     *
     * <p>Each signal is logged per module as well as summed. Only the four sums used to be logged,
     * which made a single bad module invisible: a drive motor that stopped turning its wheel showed
     * up as a dip in a total that the other three still dominated. The per-module keys cost twelve
     * more doubles on a tick that has already paid for the refresh.
     */
    protected void logBatteryUsage() {
        if (Telemetry.slowLogThisLoop() && moduleCurrentSignals.length > 0) {
            BaseStatusSignal.refreshAll(moduleCurrentSignals);
            driveStatorCurrent = 0;
            driveSupplyCurrent = 0;
            steerStatorCurrent = 0;
            steerSupplyCurrent = 0;
            for (int i = 0; i < moduleCurrentSignals.length; i += 4) {
                double driveStator = moduleCurrentSignals[i].getValueAsDouble();
                double driveSupply = moduleCurrentSignals[i + 1].getValueAsDouble();
                double steerStator = moduleCurrentSignals[i + 2].getValueAsDouble();
                double steerSupply = moduleCurrentSignals[i + 3].getValueAsDouble();

                driveStatorCurrent += driveStator;
                driveSupplyCurrent += driveSupply;
                steerStatorCurrent += steerStator;
                steerSupplyCurrent += steerSupply;

                Telemetry.log(moduleCurrentKeys[i], driveStator);
                Telemetry.log(moduleCurrentKeys[i + 1], driveSupply);
                Telemetry.log(moduleCurrentKeys[i + 2], steerStator);
                Telemetry.log(moduleCurrentKeys[i + 3], steerSupply);
                // A signal whose refresh failed is a motor that did not answer.
                Telemetry.log(
                        moduleConnectedKeys[i / 2], moduleCurrentSignals[i].getStatus().isOK());
                Telemetry.log(
                        moduleConnectedKeys[i / 2 + 1],
                        moduleCurrentSignals[i + 2].getStatus().isOK());
            }
            Telemetry.log(CURRENTS_PREFIX + "DriveStatorCurrent", driveStatorCurrent);
            Telemetry.log(CURRENTS_PREFIX + "SteerStatorCurrent", steerStatorCurrent);
            Telemetry.log(CURRENTS_PREFIX + "DriveSupplyCurrent", driveSupplyCurrent);
            Telemetry.log(CURRENTS_PREFIX + "SteerSupplyCurrent", steerSupplyCurrent);
        }
        Robot.getBatteryLogger().reportCurrentUsage("Mechanisms/SwerveSteer", steerSupplyCurrent);
        Robot.getBatteryLogger().reportCurrentUsage("Mechanisms/SwerveDrive", driveSupplyCurrent);
    }

    /**
     * This method is called periodically and is used to update the pilot's perspective. It ensures
     * that the swerve drive system is aligned correctly based on the pilot's view.
     */
    @Override
    public void periodic() {
        systemState = handleStateTransition();
        applyStates();

        Telemetry.logState("Swerve/WantedState", wantedState);
        Telemetry.logState("Swerve/SystemState", systemState);
        Telemetry.log("Swerve/CurrentCommand", getCurrentCommandName());
        Telemetry.log("Swerve/TeleopVelocityCoefficient", getTeleopVelocityCoefficient());
        Telemetry.log(
                "Swerve/TeleopRotationVelocityCoefficient", getTeleopRotationVelocityCoefficient());
        logSwerveState();
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
        return loopState().Pose;
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
        invalidateLoopState();
    }

    @Override
    public void resetTranslation(Translation2d translation) {
        super.resetTranslation(translation);
        invalidateLoopState();
    }

    @Override
    public void resetRotation(Rotation2d rotation) {
        super.resetRotation(rotation);
        invalidateLoopState();
    }

    @Override
    public void seedFieldCentric() {
        super.seedFieldCentric();
        invalidateLoopState();
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPose, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPose, timestampSeconds);
        invalidateLoopState();
    }

    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPose, double timestampSeconds, Matrix<N3, N1> visionStdDevs) {
        super.addVisionMeasurement(visionRobotPose, timestampSeconds, visionStdDevs);
        invalidateLoopState();
    }

    // --------------------------------------------------------------------------------
    // Zone Triggers
    // --------------------------------------------------------------------------------
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
     * Returns the current robot chassis speeds.
     *
     * @return the current robot chassis speeds
     */
    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return getKinematics().toChassisSpeeds(loopState().ModuleStates);
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

    // ── Public state setters ───────────────────────────────────────────────────────────
    /**
     * Sets the wanted state.
     *
     * @param state the wanted state
     */
    public void setWantedState(WantedState state) {
        this.wantedState = state;
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
