package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
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
import frc.rebuilt.ShotCalculator;
import frc.robot.Robot;
import frc.spectrumLib.swerve.MapleSimSwerveDrivetrain;
import frc.spectrumLib.telemetry.Telemetry;
import frc.spectrumLib.util.Util;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;

/** Phoenix SwerveDrivetrain wired up as a WPILib subsystem. */
public class Swerve extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {

    public enum WantedState {
        TELEOP_DRIVE,
        PILOT_AIM_AT_TARGET,
        X_BRAKE,
        IDLE
    }

    public enum SystemState {
        TELEOP_DRIVE,
        PILOT_AIM_AT_TARGET,
        X_BRAKE,
        IDLE
    }

    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLE;

    private final StatusSignal<AngularVelocity> angularPitchVelocity;
    private final StatusSignal<AngularVelocity> angularRollVelocity;
    private final StatusSignal<AngularVelocity> angularYawVelocity;
    private final StatusSignal<Angle> roll;
    private final StatusSignal<Angle> pitch;
    private final StatusSignal<LinearAcceleration> accelerationX;
    private final StatusSignal<LinearAcceleration> accelerationY;

    public static final double TRANSLATION_ERROR_MARGIN_METERS = Units.inchesToMeters(1.0);
    public static final double DRIVE_TO_POINT_STATIC_FRICTION_CONSTANT = 0.02;
    private static final double SKEW_COMPENSATION_SCALAR = -0.03;

    @Getter @Setter private double teleopVelocityCoefficient = 1.0;
    @Getter @Setter private double rotationVelocityCoefficient = 1.0;

    @Getter private SwerveConfig config;
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

    private final SwerveRequest.FieldCentricFacingAngle DRIVE_AT_ANGLE_REQUEST =
            new SwerveRequest.FieldCentricFacingAngle()
                    .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                    .withSteerRequestType(SwerveModule.SteerRequestType.Position);

    private final SwerveRequest.SwerveDriveBrake X_BRAKE = new SwerveRequest.SwerveDriveBrake();

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

        angularPitchVelocity = getPigeon2().getAngularVelocityYWorld();
        angularRollVelocity = getPigeon2().getAngularVelocityXWorld();
        angularYawVelocity = getPigeon2().getAngularVelocityZWorld();
        roll = getPigeon2().getRoll();
        pitch = getPigeon2().getPitch();
        accelerationX = getPigeon2().getAccelerationX();
        accelerationY = getPigeon2().getAccelerationY();

        BaseStatusSignal.setUpdateFrequencyForAll(250, angularYawVelocity);
        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                angularPitchVelocity,
                angularRollVelocity,
                roll,
                pitch,
                accelerationX,
                accelerationY);

        getPigeon2().optimizeBusUtilization();
        optimizeBusUtilization();

        registerTelemetry(this::log);

        Telemetry.print(getName() + " Subsystem Initialized");
    }

    protected void log(SwerveDriveState state) {
        Telemetry.log("Swerve/State/Pose", state.Pose);
        Telemetry.log("Swerve/State/TargetStates", state.ModuleTargets);
        Telemetry.log("Swerve/State/MeasuredStates", state.ModuleStates);
        Telemetry.log("Swerve/State/MeasuredSpeeds", state.Speeds);
    }

    protected void logBatteryUsage() {
        double steerMotorCurrent = getSteerMotorSupplyCurrents();
        double driveMotorCurrent = getDriveMotorSupplyCurrents();
        Robot.getBatteryLogger().reportCurrentUsage("Mechanisms/SwerveSteer", steerMotorCurrent);
        Robot.getBatteryLogger().reportCurrentUsage("Mechanisms/SwerveDrive", driveMotorCurrent);

        Telemetry.log("Swerve/Currents/DriveStatorCurrent", getDriveMotorStatorCurrents());
        Telemetry.log("Swerve/Currents/SteerStatorCurrent", getSteerMotorStatorCurrents());
        Telemetry.log("Swerve/Currents/DriveSupplyCurrent", getDriveMotorSupplyCurrents());
        Telemetry.log("Swerve/Currents/SteerSupplyCurrent", getSteerMotorSupplyCurrents());
    }

    protected double getDriveMotorStatorCurrents() {
        return Arrays.stream(getModules())
                .mapToDouble(module -> module.getDriveMotor().getStatorCurrent().getValueAsDouble())
                .sum();
    }

    protected double getSteerMotorStatorCurrents() {
        return Arrays.stream(getModules())
                .mapToDouble(module -> module.getSteerMotor().getStatorCurrent().getValueAsDouble())
                .sum();
    }

    protected double getDriveMotorSupplyCurrents() {
        return Arrays.stream(getModules())
                .mapToDouble(module -> module.getDriveMotor().getSupplyCurrent().getValueAsDouble())
                .sum();
    }

    protected double getSteerMotorSupplyCurrents() {
        return Arrays.stream(getModules())
                .mapToDouble(module -> module.getSteerMotor().getSupplyCurrent().getValueAsDouble())
                .sum();
    }

    /** Advances the state machine and the sim pose. WPILib calls this every 20 ms. */
    @Override
    public void periodic() {
        systemState = handleStateTransition();
        applyStates();

        Telemetry.log("Swerve/CurrentCommand", getCurrentCommandName());
        Telemetry.log("Swerve/TeleopVelocityCoefficient", getTeleopVelocityCoefficient());
        Telemetry.log("Swerve/RotationVelocityCoefficient", getRotationVelocityCoefficient());
        logBatteryUsage();
        checkPigeonConnection();

        if (Utils.isSimulation()) {
            Telemetry.log("Sim/SimPose", getRobotPose());
            if (robotBumpSim != null) {
                synchronized (simLock) {
                    Pose2d simPose =
                            mapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose();
                    ChassisSpeeds robotRelSpeeds =
                            mapleSimSwerveDrivetrain.mapleSimDrive
                                    .getDriveTrainSimulatedChassisSpeedsRobotRelative();
                    ChassisSpeeds fieldRelSpeeds =
                            ChassisSpeeds.fromRobotRelativeSpeeds(
                                    robotRelSpeeds, simPose.getRotation());
                    // 5 subticks gives 4 ms sub-steps, close to MapleSim's 5 ms period
                    simRobotPose3d = robotBumpSim.update(simPose, fieldRelSpeeds, 5);
                    if (robotBumpSim.isOnRamp()) {
                        mapleSimSwerveDrivetrain.mapleSimDrive.setSimulationWorldPose(
                                robotBumpSim.getSimWorldPose(simPose));
                    }
                    Telemetry.log("Sim/RobotPose3d", simRobotPose3d);
                }
            }
        }

        Telemetry.log("Swerve/WantedState", wantedState.toString());
        Telemetry.log("Swerve/SystemState", systemState.toString());
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
            case PILOT_AIM_AT_TARGET -> SystemState.PILOT_AIM_AT_TARGET;
            case X_BRAKE -> SystemState.X_BRAKE;
            case IDLE -> SystemState.IDLE;
            default -> SystemState.IDLE;
        };
    }

    private void applyStates() {
        switch (systemState) {
            default:
            case IDLE:
                setControl(new SwerveRequest.Idle());
                break;
            case PILOT_AIM_AT_TARGET:
                var params = ShotCalculator.getInstance().getParameters();
                ChassisSpeeds joystickSpeeds = calculateSpeedsBasedOnJoystickInputs();
                setControl(
                        DRIVE_AT_ANGLE_REQUEST
                                .withVelocityX(joystickSpeeds.vxMetersPerSecond)
                                .withVelocityY(joystickSpeeds.vyMetersPerSecond)
                                .withTargetDirection(params.driveAngle()));

                break;
            case TELEOP_DRIVE:
                setControl(FIELD_CENTRIC_DRIVE.withSpeeds(calculateSpeedsBasedOnJoystickInputs()));
                break;
            case X_BRAKE:
                setControl(X_BRAKE);
        }
    }

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
        double angularVelocity = angularMagnitude * rotationVelocityCoefficient;

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

    /** Returns the simulated pose in sim, otherwise the pose odometry reports. */
    public Pose2d getRobotPose() {
        if (this.mapleSimSwerveDrivetrain != null) {
            synchronized (simLock) {
                return mapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose();
            }
        }
        return getState().Pose;
    }

    /** Raises the alert that surfaces an unconnected Pigeon IMU in Elastic. */
    private void checkPigeonConnection() {
        if (getPigeon2() == null || !getPigeon2().isConnected()) {
            pigeonAlert.set(true);
        } else {
            pigeonAlert.set(false);
        }
    }

    /**
     * Samples the pose buffer at a timestamp.
     *
     * @param timestampSeconds a RoboRIO timestamp, as in {@link Utils#fpgaToCurrentTime}
     * @return the sampled pose, or the current pose if the timestamp is not buffered
     */
    public Pose2d getPoseAtTimestamp(double timestampSeconds) {
        Optional<Pose2d> sampled = super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));

        return sampled.orElse(getRobotPose());
    }

    @Override
    public void resetPose(Pose2d pose) {
        if (this.mapleSimSwerveDrivetrain != null) {
            synchronized (simLock) {
                mapleSimSwerveDrivetrain.mapleSimDrive.setSimulationWorldPose(pose);
            }
            Timer.delay(0.05); // let the sim apply the new pose first
        }
        super.resetPose(pose);
    }

    public Trigger inXzone(double minXmeter, double maxXmeter) {
        return new Trigger(
                () -> Util.inRange(() -> getRobotPose().getX(), () -> minXmeter, () -> maxXmeter));
    }

    public Trigger inYzone(double minYmeter, double maxYmeter) {
        return new Trigger(
                () -> Util.inRange(() -> getRobotPose().getY(), () -> minYmeter, () -> maxYmeter));
    }

    // The Alliance variants take bounds in the blue alliance frame and flip the pose instead of the
    // bounds, so callers never have to think about which alliance they are on.
    public Trigger inXzoneAlliance(double minXmeter, double maxXmeter) {
        return new Trigger(
                () ->
                        Util.inRange(
                                FieldHelpers.flipXifRed(getRobotPose().getX()),
                                minXmeter,
                                maxXmeter));
    }

    public Trigger inYzoneAlliance(double minYmeter, double maxYmeter) {
        return new Trigger(
                () ->
                        Util.inRange(
                                FieldHelpers.flipYifRed(getRobotPose().getY()),
                                minYmeter,
                                maxYmeter));
    }

    private static final double NEUTRAL_DEPTH_METERS = Units.inchesToMeters(283.0);
    private static final double NEUTRAL_LENGTH_METERS = Units.inchesToMeters(317.7);
    private static final double ENEMY_ALLIANCE_DEPTH_METERS = Units.inchesToMeters(180.0);

    private static final Rectangle2d NEUTRAL_ZONE =
            new Rectangle2d(
                    new Translation2d(
                            Field.fieldLength / 2.0 - NEUTRAL_DEPTH_METERS / 2.0,
                            Field.fieldWidth / 2.0 - NEUTRAL_LENGTH_METERS / 2.0),
                    new Translation2d(
                            Field.fieldLength / 2.0 + NEUTRAL_DEPTH_METERS / 2.0,
                            Field.fieldWidth / 2.0 + NEUTRAL_LENGTH_METERS / 2.0));

    private static final Rectangle2d ENEMY_ALLIANCE_ZONE =
            new Rectangle2d(
                    new Translation2d(Field.fieldLength - ENEMY_ALLIANCE_DEPTH_METERS, 0),
                    new Translation2d(Field.fieldLength, Field.fieldWidth));

    /** Returns {@code true} when the robot is inside the neutral zone. Allocation-free. */
    public boolean isInNeutralZone() {
        return NEUTRAL_ZONE.contains(getRobotPose().getTranslation());
    }

    /**
     * Returns {@code true} when the robot is inside the opposing alliance's zone (pose X is flipped
     * for red so the same rectangle works for both alliances).
     */
    public boolean isInEnemyAllianceZone() {
        return ENEMY_ALLIANCE_ZONE.contains(
                new Translation2d(
                        FieldHelpers.flipXifRed(getRobotPose().getX()), getRobotPose().getY()));
    }

    public Trigger inNeutralZone() {
        return new Trigger(this::isInNeutralZone);
    }

    public Trigger inEnemyAllianceZone() {
        return new Trigger(this::isInEnemyAllianceZone);
    }

    public Trigger inFieldRight() {
        final double fieldWidthMeters = Units.feetToMeters(27.0); // full field width
        final double halfWidth = fieldWidthMeters / 2.0;

        return new Trigger(() -> getRobotPose().getY() < halfWidth);
    }

    public Trigger inFieldLeft() {
        final double fieldWidthMeters = Units.feetToMeters(27.0); // full field width
        final double halfWidth = fieldWidthMeters / 2.0;

        return new Trigger(() -> getRobotPose().getY() >= halfWidth);
    }

    public boolean isGoingTooFast(double thresholdSpeed) {
        ChassisSpeeds speeds = getCurrentRobotChassisSpeeds();
        double linearSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        return linearSpeed > thresholdSpeed;
    }

    public Trigger overSpeedTrigger(double thresholdSpeed) {
        return new Trigger(() -> isGoingTooFast(thresholdSpeed));
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return getKinematics().toChassisSpeeds(getState().ModuleStates);
    }

    private void applyReorient(double angleDegrees) {
        resetPose(
                new Pose2d(
                        getRobotPose().getX(),
                        getRobotPose().getY(),
                        Rotation2d.fromDegrees(angleDegrees)));
    }

    protected Command reorient(double angleDegrees) {
        return runOnce(() -> applyReorient(angleDegrees));
    }

    public Command reorientForward() {
        return reorient(0).withName("reorientForward");
    }

    public Command reorientLeft() {
        return reorient(90).withName("reorientLeft");
    }

    public Command reorientBack() {
        return reorient(180).withName("reorientBack");
    }

    public Command reorientRight() {
        return reorient(270).withName("reorientRight");
    }

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

    protected Command cardinalReorient() {
        return runOnce(
                () -> {
                    double angleDegrees = getClosestCardinal();
                    applyReorient(angleDegrees);
                });
    }

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

    public double getRotationDifference(double angle1, double angle2) {
        double diff = Math.abs(angle1 - angle2) % 360;
        return diff > 180 ? 360 - diff : diff;
    }

    Rotation2d getRotation() {
        return getRobotPose().getRotation();
    }

    double getRotationRadians() {
        return getRobotPose().getRotation().getRadians();
    }

    // Runs the request every cycle, even while disabled, so aiming commands stay continuous.
    Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get())).ignoringDisable(true);
    }

    public void setWantedState(WantedState state) {
        this.wantedState = state;
    }

    public boolean isAtDesiredRotation() {
        return isAtDesiredRotation(Units.degreesToRadians(10.0));
    }

    public boolean isAtDesiredRotation(double toleranceRadians) {
        if (systemState != SystemState.PILOT_AIM_AT_TARGET) {
            return false;
        }
        return Math.abs(DRIVE_AT_ANGLE_REQUEST.HeadingController.getPositionError())
                < toleranceRadians;
    }

    private void configurePathPlanner() {
        // Paths reseed this, so the pre-match pose only has to be plausible
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
                            new PIDConstants(4, 0, 0), // translation
                            new PIDConstants(3, 0, 0)), // rotation
                    config,
                    // Paths are authored against blue, so PathPlanner mirrors them for red
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                    this);
        } catch (Exception ex) {
            DriverStation.reportError(
                    "Failed to load PathPlanner config and configure AutoBuilder",
                    ex.getStackTrace());
        }
    }

    @Getter private MapleSimSwerveDrivetrain mapleSimSwerveDrivetrain = null;
    @Getter private RobotBumpSim robotBumpSim = null;
    @Getter private Pose3d simRobotPose3d = Pose3d.kZero;
    private final Object simLock = new Object();

    @SuppressWarnings("unchecked")
    private void startSimThread() {
        mapleSimSwerveDrivetrain =
                new MapleSimSwerveDrivetrain(
                        Seconds.of(config.getSimLoopPeriod()),
                        Pounds.of(115),
                        Inches.of(30),
                        Inches.of(30),
                        DCMotor.getKrakenX60Foc(1),
                        DCMotor.getKrakenX60Foc(1),
                        1.2, // wheel COF
                        getModuleLocations(),
                        getPigeon2(),
                        getModules(),
                        config.getFrontLeft(),
                        config.getFrontRight(),
                        config.getBackLeft(),
                        config.getBackRight());
        robotBumpSim = new RobotBumpSim(getModuleLocations());

        // Run faster than real time so PID gains behave more reasonably
        simNotifier =
                new Notifier(
                        () -> {
                            synchronized (simLock) {
                                mapleSimSwerveDrivetrain.update();
                            }
                        });
        simNotifier.startPeriodic(config.getSimLoopPeriod());
    }
}
