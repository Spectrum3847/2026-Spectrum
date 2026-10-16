// Based on
// https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/SwerveWithPathPlanner/src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java
package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
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

/** The drivetrain. Extends CTRE's SwerveDrivetrain and implements Subsystem. */
public class Swerve extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {

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

    /** Snapshot of the drivetrain state for this loop; see {@link #loopState()}. */
    private SwerveDriveState loopState;

    /** Loop the snapshot was taken on, or -1 for none. */
    private long loopStateLoop = -1;

    /**
     * The drivetrain state, read once per loop.
     *
     * <p>{@code getState()} is a JNI call plus the odometry lock plus a copy, and the pose gets
     * pulled that way a dozen times a loop, from vision, the shot calculator, the superstructure,
     * the turret, Field2d and the zone triggers. Every caller in a loop now sees the same snapshot,
     * which is also the right semantics: a shot solution and the turret aiming it should agree on
     * where the robot is. The snapshot is dropped whenever the pose is changed on purpose, so a
     * correction made early in the loop is seen by everything after it.
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
     * <p>Not from CTRE's {@code registerTelemetry} callback, which runs on the odometry thread
     * while it holds the drivetrain state lock. Four struct serializations and a queue put per call
     * happened under that lock, so a full DogLog queue dropped exactly these four keys, and {@code
     * Swerve.periodic} then showed up at 100 to 200 ms in overrun traces waiting on that lock from
     * {@code setControl}. Logging the loop's own snapshot here costs the odometry thread nothing
     * and keeps odometry at 250 Hz.
     */
    private void logSwerveState() {
        SwerveDriveState state = loopState();
        Telemetry.log("Swerve/State/Pose", state.Pose);
        Telemetry.log("Swerve/State/TargetStates", state.ModuleTargets);
        Telemetry.log("Swerve/State/MeasuredStates", state.ModuleStates);
        Telemetry.log("Swerve/State/MeasuredSpeeds", state.Speeds);
    }

    /** NetworkTables/DogLog key prefix for the drivetrain's current telemetry. */
    private static final String CURRENTS_PREFIX = "Swerve/Currents/";

    /** Drive and steer stator and supply current signals for every module, refreshed together. */
    private BaseStatusSignal[] moduleCurrentSignals = new BaseStatusSignal[0];

    /** Log key for each entry of {@link #moduleCurrentSignals}, in the same order. */
    private String[] moduleCurrentKeys = new String[0];

    /**
     * {@code Swerve/Modules/<name>/DriveConnected} and {@code .../SteerConnected}, two per module.
     *
     * <p>The only other connection telemetry the swerve has is {@code Swerve/Align/.../Connected},
     * which publishes while disabled, so a drivetrain motor going quiet mid-match leaves no direct
     * evidence and the failure has to be read off its currents at 10 Hz. These ride the same 10 Hz
     * refresh as the currents.
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
     * held between ticks. The battery logger's energy integral tolerates a 100 ms sample and hold.
     *
     * <p>Each signal is logged per module as well as summed, because a drive motor that stopped
     * turning its wheel otherwise shows up only as a dip in a total the other three still dominate.
     * The per-module keys cost twelve more doubles on a tick that has already paid for the refresh.
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
                // subticks 5 gives dt = 20ms/5 = 4ms sub-steps, close to MapleSim's 5ms period
                simRobotPose3d = robotBumpSim.update(simPose, fieldRelSpeeds, 5);
                if (robotBumpSim.isOnRamp()) {
                    mapleSimSwerveDrivetrain.mapleSimDrive.setSimulationWorldPose(
                            robotBumpSim.getSimWorldPose(simPose));
                }
                Telemetry.log("Sim/RobotPose3d", simRobotPose3d);
            }
        }
    }

    protected String getCurrentCommandName() {
        Command currentCommand = this.getCurrentCommand();
        if (currentCommand != null) {
            return currentCommand.getName();
        }

        return "none";
    }

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case TELEOP_DRIVE -> SystemState.TELEOP_DRIVE;
            case X_BRAKE -> SystemState.X_BRAKE;
            case IDLE -> SystemState.IDLE;
        };
    }

    private void applyStates() {
        switch (systemState) {
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

    private ChassisSpeeds calculateSpeedsBasedOnJoystickInputs() {
        if (DriverStation.getAlliance().isEmpty()) {
            return new ChassisSpeeds(0, 0, 0);
        }

        double xMagnitude = Robot.getPilot().getDriveFwdPositive();
        double yMagnitude = Robot.getPilot().getDriveLeftPositive();
        double angularMagnitude = Robot.getPilot().getDriveCCWPositive();

        // Field-relative stick directions are mirrored for the red alliance.
        double allianceSign = Field.isBlue() ? 1 : -1;
        double xVelocity = allianceSign * xMagnitude * teleopVelocityCoefficient;
        double yVelocity = allianceSign * yMagnitude * teleopVelocityCoefficient;
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

    public Pose2d getRobotPose() {
        // In simulation the pose comes from MapleSim, which also models collisions with the field
        // obstacles and boundaries.
        if (this.mapleSimSwerveDrivetrain != null) {
            return mapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose();
        }
        return loopState().Pose;
    }

    private void checkPigeonConnection() {
        pigeonAlert.set(!getPigeon2().isConnected());
    }

    /**
     * The interpolated pose at a timestamp, falling back to the current pose if it is not buffered.
     */
    public Pose2d getPoseAtTimestamp(double timestampSeconds) {
        Optional<Pose2d> sampled = super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));

        return sampled.orElse(getRobotPose());
    }

    @Override
    public void resetPose(Pose2d pose) {
        if (this.mapleSimSwerveDrivetrain != null) {
            mapleSimSwerveDrivetrain.mapleSimDrive.setSimulationWorldPose(pose);
            Timer.delay(0.05); // let the simulation catch up
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

    private static final double NEUTRAL_DEPTH_METERS = Units.inchesToMeters(283.0);
    private static final double ENEMY_ALLIANCE_DEPTH_METERS = Units.inchesToMeters(180.0);

    private static final Rectangle2d NEUTRAL_ZONE =
            new Rectangle2d(
                    new Translation2d(Field.fieldLength / 2.0 - NEUTRAL_DEPTH_METERS / 2.0, 0),
                    new Translation2d(
                            Field.fieldLength / 2.0 + NEUTRAL_DEPTH_METERS / 2.0,
                            Field.fieldWidth));

    private static final Rectangle2d ENEMY_ALLIANCE_ZONE =
            new Rectangle2d(
                    new Translation2d(Field.fieldLength - ENEMY_ALLIANCE_DEPTH_METERS, 0),
                    new Translation2d(Field.fieldLength, Field.fieldWidth));

    /** True when the robot is inside the neutral zone. Allocation-free. */
    public boolean isInNeutralZone() {
        return NEUTRAL_ZONE.contains(getRobotPose().getTranslation());
    }

    /**
     * True when the robot is inside the opposing alliance's zone. Pose X is flipped for red, so the
     * same rectangle works for both alliances.
     */
    public boolean isInEnemyAllianceZone() {
        Pose2d pose = getRobotPose();
        return ENEMY_ALLIANCE_ZONE.contains(
                new Translation2d(FieldHelpers.flipXifRed(pose.getX()), pose.getY()));
    }

    public Trigger inFieldLeft() {
        final double fieldWidthMeters = Units.feetToMeters(27.0); // full field width (Y)
        final double halfWidth = fieldWidthMeters / 2.0;

        return new Trigger(() -> getRobotPose().getY() >= halfWidth);
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return getKinematics().toChassisSpeeds(loopState().ModuleStates);
    }

    protected void reorient(double angleDegrees) {
        resetPose(
                new Pose2d(
                        getRobotPose().getX(),
                        getRobotPose().getY(),
                        Rotation2d.fromDegrees(angleDegrees)));
    }

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
     */
    public Command reorientForward() {
        return reorientPilotAngle(0).withName("Swerve.reorientForward");
    }

    /** Reorients the robot front to the driver's left. */
    public Command reorientLeft() {
        return reorientPilotAngle(90).withName("Swerve.reorientLeft");
    }

    /** Reorients the robot front back toward the driver station. */
    public Command reorientBack() {
        return reorientPilotAngle(180).withName("Swerve.reorientBack");
    }

    /** Reorients the robot front to the driver's right. */
    public Command reorientRight() {
        return reorientPilotAngle(270).withName("Swerve.reorientRight");
    }

    public void setWantedState(WantedState state) {
        this.wantedState = state;
    }

    private void configurePathPlanner() {
        // Seeds the robot in front of the blue hub. Paths change this starting position.
        resetPose(
                new Pose2d(
                        Field.getBlueHubCenter().getX() - 2,
                        Field.getBlueHubCenter().getY(),
                        Rotation2d.fromDegrees(0)));

        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    this::getRobotPose,
                    this::resetPose,
                    this::getCurrentRobotChassisSpeeds,
                    (speeds, feedforwards) -> {
                        setControl(
                                AutoRequest.withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))
                                        .withWheelForceFeedforwardsX(
                                                feedforwards.robotRelativeForcesX())
                                        .withWheelForceFeedforwardsY(
                                                feedforwards.robotRelativeForcesY()));
                    },
                    new PPHolonomicDriveController(
                            new PIDConstants(4, 0, 0), new PIDConstants(4, 0, 0)),
                    config,
                    // Assume the path needs to be flipped for Red vs Blue, this is normally the
                    // case
                    Field::isRed,
                    this);
        } catch (Exception ex) {
            DriverStation.reportError(
                    "Failed to load PathPlanner config and configure AutoBuilder",
                    ex.getStackTrace());
        }
    }

    // Simulated drivetrain, used for robot bump simulation.
    @Getter private MapleSimSwerveDrivetrain mapleSimSwerveDrivetrain = null;

    @Getter private RobotBumpSim robotBumpSim = null;
    @Getter private Pose3d simRobotPose3d = Pose3d.kZero;

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
