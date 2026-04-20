// Based on
// https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/SwerveWithPathPlanner/src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java
package frc.robot.swerve;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;

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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.rebuilt.Field;
import frc.rebuilt.FieldHelpers;
import frc.robot.Robot;
import frc.robot.swerve.controllers.RotationController;
import frc.robot.swerve.controllers.TranslationXController;
import frc.robot.swerve.controllers.TranslationYController;
import frc.spectrumLib.SpectrumSubsystem;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.swerve.MapleSimSwerveDrivetrain;
import frc.spectrumLib.util.Util;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
public class Swerve extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder>
        implements SpectrumSubsystem {
    @Getter private SwerveConfig config;
    private Notifier simNotifier = null;
    private RotationController rotationController;
    private TranslationXController xController;
    private TranslationYController yController;

    private Alert pigeonAlert = new Alert("Pigeon IMU Disconnected", Alert.AlertType.kError);

    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedPilotPerspective = false;

    private final SwerveRequest.ApplyRobotSpeeds AutoRequest =
            new SwerveRequest.ApplyRobotSpeeds()
                    .withDriveRequestType(DriveRequestType.Velocity)
                    .withSteerRequestType(SteerRequestType.Position)
                    .withDesaturateWheelSpeeds(true);

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

        rotationController = new RotationController(config);
        xController = new TranslationXController(config);
        yController = new TranslationYController(config);

        if (Utils.isSimulation()) {
            startSimThread();
        }

        configurePathPlanner();

        Robot.add(this);
        this.register();

        registerTelemetry(this::log);

        Telemetry.print(getName() + " Subsystem Initialized: ");
    }

    // --------------------------------------------------------------------------------
    // Periodic and Setup Methods
    // --------------------------------------------------------------------------------

    protected void log(SwerveDriveState state) {
        Telemetry.log("Swerve/Pose", state.Pose);
        Telemetry.log("Swerve/TargetStates", state.ModuleTargets);
        Telemetry.log("Swerve/MeasuredStates", state.ModuleStates);
        Telemetry.log("Swerve/MeasuredSpeeds", state.Speeds);
        Telemetry.log("Swerve/DriveStatorCurrent", getDriveMotorStatorCurrents());
        Telemetry.log("Swerve/SteerStatorCurrent", getSteerMotorStatorCurrents());
        Telemetry.log("Swerve/DriveSupplyCurrent", getDriveMotorSupplyCurrents());
        Telemetry.log("Swerve/SteerSupplyCurrent", getSteerMotorSupplyCurrents());
    }

    protected void logBatteryUsage() {
        double steerMotorCurrent = getSteerMotorSupplyCurrents();
        double driveMotorCurrent = getDriveMotorSupplyCurrents();
        Robot.getBatteryLogger().reportCurrentUsage("Mechanisms/SwerveSteer", steerMotorCurrent);
        Robot.getBatteryLogger().reportCurrentUsage("Mechanisms/SwerveDrive", driveMotorCurrent);
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

    /**
     * This method is called periodically and is used to update the pilot's perspective. It ensures
     * that the swerve drive system is aligned correctly based on the pilot's view.
     */
    @Override
    public void periodic() {
        Telemetry.log("Swerve/CurrentCommand", getCurrentCommandName());
        logBatteryUsage();
        setPilotPerspective();
        checkPigeonConnection();
    }

    @Override
    public void setupStates() {
        SwerveStates.setStates();
    }

    @Override
    public void setupDefaultCommand() {
        SwerveStates.setupDefaultCommand();
    }

    protected String getCurrentCommandName() {
        Command currentCommand = this.getCurrentCommand();
        if (currentCommand != null) {
            return currentCommand.getName();
        }

        return "none";
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
        if (getPigeon2() == null || !getPigeon2().isConnected()) {
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

    public Trigger inXzone(double minXmeter, double maxXmeter) {
        return new Trigger(
                () -> Util.inRange(() -> getRobotPose().getX(), () -> minXmeter, () -> maxXmeter));
    }

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

    public Trigger inNeutralZone() {
        final double fieldLengthMeters = Units.feetToMeters(54.0);
        final double fieldWidthMeters = Units.feetToMeters(27.0);

        final double neutralDepthMeters = Units.inchesToMeters(283.0);
        final double neutralLengthMeters = Units.inchesToMeters(317.7);

        final double centerX = fieldLengthMeters / 2.0;
        final double centerY = fieldWidthMeters / 2.0;

        Rectangle2d neutralZone =
                new Rectangle2d(
                        new Translation2d(
                                (centerX - neutralDepthMeters / 2.0),
                                centerY - neutralLengthMeters / 2.0),
                        new Translation2d(
                                (centerX + neutralDepthMeters / 2.0),
                                centerY + neutralLengthMeters / 2.0));

        return new Trigger(
                () -> {
                    double x = getRobotPose().getX();
                    double y = getRobotPose().getY();

                    return neutralZone.contains(new Translation2d(x, y));
                });
    }

    public Trigger inEnemyAllianceZone() {
        final double fieldLengthMeters = Units.feetToMeters(54.0);
        final double fieldWidthMeters = Units.feetToMeters(27.0);

        final double allianceDepthMeters = Units.inchesToMeters(180.0); // X depth
        final double allianceSpanMeters = fieldWidthMeters; // Y span

        final double minX = fieldLengthMeters - allianceDepthMeters;
        final double minY = 0;

        Rectangle2d enemyAllianceZone =
                new Rectangle2d(
                        new Translation2d(minX, minY),
                        new Translation2d(fieldLengthMeters, allianceSpanMeters));

        return new Trigger(
                () -> {
                    double x = FieldHelpers.flipXifRed(getRobotPose().getX());
                    double y = getRobotPose().getY();

                    return enemyAllianceZone.contains(new Translation2d(x, y));
                });
    }

    public Trigger inFieldRight() {
        final double fieldWidthMeters = Units.feetToMeters(27.0); // full field width (Y)
        final double halfWidth = fieldWidthMeters / 2.0;

        return new Trigger(() -> getRobotPose().getY() < halfWidth);
    }

    public Trigger inFieldLeft() {
        final double fieldWidthMeters = Units.feetToMeters(27.0); // full field width (Y)
        final double halfWidth = fieldWidthMeters / 2.0;

        return new Trigger(() -> getRobotPose().getY() >= halfWidth);
    }

    // --------------------------------------------------------------------------------
    // Speed Checks
    // --------------------------------------------------------------------------------

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

    // --------------------------------------------------------------------------------
    // Pilot Perspective
    // --------------------------------------------------------------------------------

    private void setPilotPerspective() {
        /* Periodically try to apply the operator perspective */
        /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
        /* This allows us to correct the perspective in case the robot code restarts mid-match */
        /* Otherwise, only check and apply the operator perspective if the DS is disabled */
        /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/
        if (!hasAppliedPilotPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance()
                    .ifPresent(
                            allianceColor -> {
                                this.setOperatorPerspectiveForward(
                                        allianceColor == Alliance.Red
                                                ? config.getRedAlliancePerspectiveRotation()
                                                : config.getBlueAlliancePerspectiveRotation());
                                hasAppliedPilotPerspective = true;
                            });
        }
    }

    // --------------------------------------------------------------------------------
    // Reorientation Methods
    // --------------------------------------------------------------------------------

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

    protected double getClosest45() {
        double angleRadians = getRotation().getRadians();
        double angleDegrees = Math.toDegrees(angleRadians);

        // Normalize the angle to be within 0 to 360 degrees
        angleDegrees = angleDegrees % 360;
        if (angleDegrees < 0) {
            angleDegrees += 360;
        }

        // Round to the nearest multiple of 45 degrees
        double closest45Degrees = Math.round(angleDegrees / 45.0) * 45.0;

        // Convert back to radians and return as a Rotation2d
        return Rotation2d.fromDegrees(closest45Degrees).getRadians();
    }

    protected double getClosestFieldAngle() {
        // Step 1: Read the angle in radians
        double angleRadians = getRotation().getRadians();

        // Step 2: Convert the angle from radians to degrees
        double angleDegrees = Math.toDegrees(angleRadians);

        // Step 3: Define a table of angles in degrees
        double[] angleTable = {0, 180, 126, -126, 54, -54, 60, -60, 120, -120, 90, -90};

        // Step 4: Find the nearest angle from the table
        double closestAngle = angleTable[0];
        double minDifference = getRotationDifference(angleDegrees, closestAngle);

        for (double angle : angleTable) {
            double difference = getRotationDifference(angleDegrees, angle);
            if (difference < minDifference) {
                minDifference = difference;
                closestAngle = angle;
            }
        }

        // Step 5: Return the nearest angle in Radians
        return Math.toRadians(closestAngle);
    }

    protected Command cardinalReorient() {
        return runOnce(
                () -> {
                    double angleDegrees = getClosestCardinal();
                    reorient(angleDegrees);
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

    // Helper method to calculate the shortest angle difference
    public double getRotationDifference(double angle1, double angle2) {
        double diff = Math.abs(angle1 - angle2) % 360;
        return diff > 180 ? 360 - diff : diff;
    }

    // --------------------------------------------------------------------------------
    // Rotation Controller
    // --------------------------------------------------------------------------------

    double getRotationControl(double goalRadians) {
        return rotationController.calculate(goalRadians, getRotationRadians());
    }

    void resetRotationController() {
        rotationController.reset(getRotationRadians());
    }

    Rotation2d getRotation() {
        return getRobotPose().getRotation();
    }

    double getRotationRadians() {
        return getRobotPose().getRotation().getRadians();
    }

    double calculateRotationController(DoubleSupplier targetRadians, boolean useHold) {
        return rotationController.calculate(
                targetRadians.getAsDouble(), getRotationRadians(), useHold);
    }

    // --------------------------------------------------------------------------------
    // Translation X Controller
    // --------------------------------------------------------------------------------

    void resetXController() {
        xController.reset(getRobotPose().getX());
    }

    DoubleSupplier calculateXController(DoubleSupplier targetMeters) {
        return () -> xController.calculate(targetMeters.getAsDouble(), getRobotPose().getX());
    }

    // --------------------------------------------------------------------------------
    // Translation Y Controller
    // --------------------------------------------------------------------------------

    void resetYController() {
        yController.reset(getRobotPose().getY());
    }

    DoubleSupplier calculateYController(DoubleSupplier targetMeters) {
        return () -> yController.calculate(targetMeters.getAsDouble(), getRobotPose().getY());
    }

    // --------------------------------------------------------------------------------
    // Request Methods
    // --------------------------------------------------------------------------------

    // Used to set a control request to the swerve module, ignores disable so commands are
    // continuous.
    Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get())).ignoringDisable(true);
    }

    // --------------------------------------------------------------------------------
    // Path Planner Configuration
    // --------------------------------------------------------------------------------

    private void configurePathPlanner() {
        // Seed robot to mid field at start (Paths will change this starting position)
        resetPose(Field.getCenterField());

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
                            new PIDConstants(6, 0, 0)),
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

    @Getter private MapleSimSwerveDrivetrain mapleSimSwerveDrivetrain = null;

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
        /* Run simulation at a faster rate so PID gains behave more reasonably */
        simNotifier = new Notifier(mapleSimSwerveDrivetrain::update);
        simNotifier.startPeriodic(config.getSimLoopPeriod());
    }
}
