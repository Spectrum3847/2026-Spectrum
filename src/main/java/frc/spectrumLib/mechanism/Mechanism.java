package frc.spectrumLib.mechanism;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.spectrumLib.framework.RobotLoop;
import frc.spectrumLib.hardware.CanConfigBudget;
import frc.spectrumLib.hardware.TalonFXFactory;
import frc.spectrumLib.telemetry.Telemetry;
import frc.spectrumLib.util.CanDeviceId;
import frc.spectrumLib.util.Conversions;
import java.util.function.DoubleSupplier;
import lombok.*;

/**
 * Base class for a CTRE TalonFX-driven mechanism: motor creation, cached sensor reads, control mode
 * helpers, threshold triggers, and periodic current reporting.
 *
 * <p>With {@link Config#isAttached()} false, the mechanism builds no hardware, commands nothing,
 * and every sensor read returns 0.
 *
 * <p>{@link #target} and {@link #velocityTarget} hold the last setpoint this class sent to the
 * motor, which is what the at-target triggers compare against.
 *
 * <p>Everything here is in CTRE Phoenix 6 units, so rotations and rotations per second rather than
 * degrees, and the mechanism's gearing is {@code
 * Config.talonConfig.Feedback.SensorToMechanismRatio}. A subclass supplies a {@link Config} holding
 * its motor IDs, Talon settings, followers, and rotation bounds.
 */
public abstract class Mechanism implements Subsystem {

    /** Leader TalonFX, the motor this mechanism commands. */
    @Getter protected TalonFX motor;

    /** Follower TalonFXs, empty when the mechanism runs on the leader alone. */
    @Getter protected TalonFX[] followerMotors;

    /** Motor IDs, Talon settings, and mechanism parameters. */
    public Config config;

    /** Raised by the diagnostic commands when a current check fails. */
    Alert currentAlert = new Alert("", AlertType.kWarning);

    /** One alert per follower, raised when it stops pulling its share of the load. */
    private Alert[] followerAlerts = new Alert[0];

    /**
     * Seconds a follower has drawn nothing while its leader was loaded. Any draw at all resets it.
     */
    private double[] followerMismatchSeconds = new double[0];

    /** FPGA time of the previous follower check, used to measure the mismatch interval. */
    private double followerCheckLastSeconds = 0;

    /** The last closed-loop position setpoint (in rotations) sent to the motor. */
    private double target = 0;

    /** The last closed-loop velocity setpoint (in rotations per second) sent to the motor. */
    private double velocityTarget = 0;

    // Status signals the getters read. They are refreshed together once per loop; see signalValue
    private StatusSignal<Angle> positionStatusSignal;

    private StatusSignal<AngularVelocity> velocityStatusSignal;
    private BaseStatusSignal voltageSignal;
    private BaseStatusSignal statorCurrentSignal;
    private BaseStatusSignal supplyCurrentSignal;
    private BaseStatusSignal tempSignal;
    private BaseStatusSignal[] followerSupplySignals = new BaseStatusSignal[0];

    /** Every signal above, so one refreshAll covers the loop. Empty when unattached. */
    private BaseStatusSignal[] loopSignals = new BaseStatusSignal[0];

    /** Loop on which {@link #loopSignals} were last refreshed, or -1 for never. */
    private long lastSignalRefreshLoop = -1;

    /** Per-follower supply current log keys, built once. */
    private String[] followerCurrentKeys = new String[0];

    // Diagnostic log keys, built on the first logDiagnostics() call for the prefix it was given
    private String diagnosticsPrefix;
    private String voltageKey;
    private String statorCurrentKey;
    private String supplyCurrentKey;
    private String tempKey;
    private String motorConnectedKey;

    // logStandard() keys, built the same way
    private String standardPrefix;
    private String currentCommandKey;
    private String rpmKey;

    /**
     * BatteryLogger channel name for this mechanism, built once to avoid per-loop concatenation.
     */
    private final String batteryKey;

    // Every signal published here costs CANivore bandwidth on every mechanism motor AND every
    // follower. Publishing all eight at 250 Hz put bus utilization at 66-81% in the 2026-09-04
    // logs, which is where the stale-frame warnings and the intermittently unresponsive hood came
    // from. Only the feedback used for control needs to be fast; everything else is read once per
    // 20 ms robot loop for logging, so a faster frame is bandwidth spent on samples nobody reads.

    /**
     * Rate for a leader's position and velocity. Nothing on the roboRIO consumes them faster than
     * the 50 Hz loop, since Motion Magic closes its loop on the motor itself, so 100 Hz leaves a 2x
     * margin for latency compensation. At 250 Hz the 2026-09-05 logs showed CANivore utilization at
     * 63-77% and the roboRIO CPU at 92-95%.
     */
    private static final double CONTROL_SIGNAL_HZ = 100;

    /**
     * Rate for the output signals (duty cycle, motor voltage, torque current) of a leader that has
     * followers. A follower mirrors its leader out of the leader's status frames, so CTRE requires
     * these to stay enabled on such a leader. 50 Hz is the rate the followers have run at.
     */
    private static final double FOLLOWED_LEADER_OUTPUT_HZ = 50;

    /**
     * Rate for the signals nothing controls on: currents and voltage, the output signals of a
     * leader with no followers, and everything on a follower. Read once per loop and logged at 10
     * Hz, so 20 Hz is already double what gets kept.
     */
    private static final double DIAGNOSTIC_SIGNAL_HZ = 20;

    /** Rate for device temperature, which changes over seconds. */
    private static final double TEMPERATURE_SIGNAL_HZ = 4;

    /** What a motor has to publish, by its job in the mechanism. */
    private enum SignalRole {
        /** Runs the mechanism alone. */
        LEADER,
        /** Runs the mechanism with followers mirroring its output frames. */
        FOLLOWED_LEADER,
        /** Mirrors a leader; nothing on the rio reads its feedback. */
        FOLLOWER
    }

    /**
     * Builds the leader TalonFX and any followers if {@link Config#isAttached()} is true. Sensor
     * caches exist either way, so an unattached mechanism reads 0.
     */
    protected Mechanism(Config config) {
        this.config = config;
        batteryKey = "Mechanisms/" + config.getName();

        if (isAttached()) {
            motor = TalonFXFactory.createConfigTalon(config.id, config.talonConfig);
            boolean hasFollowers = config.followerConfigs.length > 0;
            configureStatusSignals(
                    motor,
                    hasFollowers ? SignalRole.FOLLOWED_LEADER : SignalRole.LEADER,
                    config.fastOutputLogging);

            followerMotors = new TalonFX[config.followerConfigs.length];
            followerSupplySignals = new BaseStatusSignal[config.followerConfigs.length];
            followerCurrentKeys = new String[config.followerConfigs.length];
            for (int i = 0; i < config.followerConfigs.length; i++) {
                followerMotors[i] =
                        TalonFXFactory.createPermanentFollowerTalon(
                                config.followerConfigs[i].id,
                                motor,
                                config.followerConfigs[i].opposeLeader);
                configureStatusSignals(followerMotors[i], SignalRole.FOLLOWER, false);
                followerSupplySignals[i] = followerMotors[i].getSupplyCurrent(false);
                followerCurrentKeys[i] =
                        "Followers/" + config.followerConfigs[i].getName() + "/SupplyCurrent";
            }

            // getX(false) hands back the device's signal object without refreshing it. The
            // getters refresh all of them in one Phoenix call per loop.
            positionStatusSignal = motor.getPosition(false);
            velocityStatusSignal = motor.getVelocity(false);
            voltageSignal = motor.getMotorVoltage(false);
            statorCurrentSignal = motor.getStatorCurrent(false);
            supplyCurrentSignal = motor.getSupplyCurrent(false);
            tempSignal = motor.getDeviceTemp(false);
            loopSignals = new BaseStatusSignal[6 + followerSupplySignals.length];
            loopSignals[0] = positionStatusSignal;
            loopSignals[1] = velocityStatusSignal;
            loopSignals[2] = voltageSignal;
            loopSignals[3] = statorCurrentSignal;
            loopSignals[4] = supplyCurrentSignal;
            loopSignals[5] = tempSignal;
            System.arraycopy(
                    followerSupplySignals, 0, loopSignals, 6, followerSupplySignals.length);

            followerAlerts = new Alert[config.followerConfigs.length];
            followerMismatchSeconds = new double[config.followerConfigs.length];
            for (int i = 0; i < config.followerConfigs.length; i++) {
                FollowerConfig f = config.followerConfigs[i];
                followerAlerts[i] =
                        new Alert(
                                f.getName()
                                        + " (CAN "
                                        + f.getId().getDeviceNumber()
                                        + ") is not drawing current while "
                                        + config.getName()
                                        + " is loaded - check its power leads",
                                AlertType.kError);
            }
        }

        this.register();
    }

    /**
     * Overrides the {@code attached} flag in the config before the hardware is built.
     *
     * @param attached false to run the mechanism in software-only mode
     */
    protected Mechanism(Config config, boolean attached) {
        // The override has to land before the delegated constructor runs, because that constructor
        // reads the flag to decide whether to create the motor.
        this(applyAttachedOverride(config, attached));
    }

    private static Config applyAttachedOverride(Config config, boolean attached) {
        config.attached = attached;
        return config;
    }

    /**
     * Sets the status frame rates this mechanism relies on, then lets Phoenix disable everything
     * else. A follower's frames cost the same bandwidth as the leader's, so it runs at the
     * diagnostic rate throughout. A leader with followers keeps the output frames they mirror. A
     * leader whose feedforward is fit from logs ({@link Config#isFastOutputLogging()}) publishes
     * its output at the control rate, so the logged voltage lines up with the logged velocity.
     */
    private static void configureStatusSignals(TalonFX talon, SignalRole role, boolean fastOutput) {
        double controlHz = role == SignalRole.FOLLOWER ? DIAGNOSTIC_SIGNAL_HZ : CONTROL_SIGNAL_HZ;
        double outputHz =
                fastOutput
                        ? CONTROL_SIGNAL_HZ
                        : role == SignalRole.FOLLOWED_LEADER
                                ? FOLLOWED_LEADER_OUTPUT_HZ
                                : DIAGNOSTIC_SIGNAL_HZ;
        BaseStatusSignal.setUpdateFrequencyForAll(
                controlHz, talon.getPosition(), talon.getVelocity());
        BaseStatusSignal.setUpdateFrequencyForAll(
                outputHz, talon.getDutyCycle(), talon.getMotorVoltage(), talon.getTorqueCurrent());
        BaseStatusSignal.setUpdateFrequencyForAll(
                DIAGNOSTIC_SIGNAL_HZ, talon.getStatorCurrent(), talon.getSupplyCurrent());
        talon.getDeviceTemp().setUpdateFrequency(TEMPERATURE_SIGNAL_HZ);
        // A long run of per-signal config calls, and only ever an optimization. On a dead bus it
        // is pure boot latency, so it is the first thing dropped once the budget is spent.
        if (!CanConfigBudget.exhausted()) {
            talon.optimizeBusUtilization();
        }
    }

    /**
     * Called once per scheduler loop. Subclasses override this for their state machine, telemetry
     * and sensor work.
     */
    @Override
    public void periodic() {}

    /** Called once per simulation loop, for physics model inputs. */
    @Override
    public void simulationPeriodic() {}

    /** The mechanism's name, as its {@link Config} spells it. */
    @Override
    public String getName() {
        return config.getName();
    }

    /** True when the mechanism has hardware and should send motor commands. */
    public boolean isAttached() {
        return config.isAttached();
    }

    /**
     * True when the leader is attached and its status frames are arriving over CAN. A mechanism
     * that is commanded but reports 0 V with this false has dropped off the bus, which is what the
     * hood did intermittently on the 2026-09-04 bench.
     */
    public boolean isMotorConnected() {
        return isAttached() && motor.isConnected();
    }

    /**
     * Reports the leader's and followers' combined supply current to the battery logger, and checks
     * each follower for life. Does nothing if the mechanism is not attached.
     */
    public void logBatteryUsage() {
        if (isAttached()) {
            // getSupplyCurrent() refreshes every signal for this loop, followers included.
            double motorCurrent = getSupplyCurrent();
            double followersCurrent = 0;
            for (int i = 0; i < followerMotors.length; i++) {
                double amps = followerSupplySignals[i].getValueAsDouble();
                followersCurrent += amps;
                checkFollowerAlive(i, amps, motorCurrent);
            }
            Robot.getBatteryLogger()
                    .reportCurrentUsage(batteryKey, motorCurrent + followersCurrent);
        }
    }

    /** Leader supply current above which a healthy follower should be pulling its share. */
    private static final double FOLLOWER_CHECK_LEADER_AMPS = 8.0;

    /** Follower supply current at or below which it is doing no work at all. */
    private static final double FOLLOWER_DEAD_AMPS = 0.5;

    /**
     * Seconds of accumulated mismatch before it is called out.
     *
     * <p>Accumulated rather than continuous, because nothing here stays loaded for long. A launch
     * burst runs one to three seconds, and the longest unbroken stretch of the launcher tower's
     * leader above the threshold in either 2026-09-05 log was 2.78 s, so a rule wanting an
     * uninterrupted fault window would have watched that tower run all day on one motor and said
     * nothing. Replayed against those logs, 1.5 s trips the dead follower in both while the two
     * healthy ones reach 0.00 s and 0.50 s.
     */
    private static final double FOLLOWER_DEAD_SECONDS = 1.5;

    /** Ceiling on one loop's contribution, so a disabled stretch cannot bank a fault. */
    private static final double FOLLOWER_MAX_LOOP_SECONDS = 0.5;

    /**
     * Raises an alert when a follower stops pulling while its leader is clearly working.
     *
     * <p>A dead permanent follower is invisible. It shares a gearbox with its leader, so the
     * mechanism keeps moving on one motor at half the torque and no dashboard looks wrong. On
     * 2026-09-05 the launcher tower ran the whole day that way with its second motor's power lead
     * off. Reading the gap between {@code BatteryLogger/Current/Mechanisms/*} (leader plus
     * followers) and {@code LauncherTower/SupplyCurrent} (leader only) across three logs was the
     * only place the fault showed up, so each follower now publishes its own draw and calls out its
     * own silence.
     *
     * <p>This only catches a dead power path. A follower off the CAN bus entirely never updates its
     * signal, so it reads a constant zero and trips the same way, which is the right outcome even
     * though the message names the wrong wire.
     */
    private void checkFollowerAlive(int index, double followerAmps, double leaderAmps) {
        if (index >= followerAlerts.length || followerAlerts[index] == null) {
            return;
        }
        // Per-follower keys, so a dead follower is visible on the dashboard at all.
        if (Telemetry.slowLogThisLoop()) {
            Telemetry.logDash(followerCurrentKeys[index], followerAmps, "amps");
        }

        double now = Timer.getFPGATimestamp();
        double dt = MathUtil.clamp(now - followerCheckLastSeconds, 0, FOLLOWER_MAX_LOOP_SECONDS);
        if (index == followerMotors.length - 1) {
            followerCheckLastSeconds = now;
        }

        if (followerAmps > FOLLOWER_DEAD_AMPS) {
            // Any draw at all means the power path is intact, so forget what came before it.
            followerMismatchSeconds[index] = 0;
        } else if (leaderAmps > FOLLOWER_CHECK_LEADER_AMPS) {
            followerMismatchSeconds[index] += dt;
        }

        followerAlerts[index].set(followerMismatchSeconds[index] >= FOLLOWER_DEAD_SECONDS);
    }

    /** The running command's name, or {@code "none"} when nothing is scheduled. */
    protected String getCurrentCommandName() {
        Command currentCommand = this.getCurrentCommand();
        if (currentCommand != null) {
            return currentCommand.getName();
        }
        return "none";
    }

    public Trigger runningDefaultCommand() {
        return new Trigger(this::isRunningDefaultCommand);
    }

    private boolean isRunningDefaultCommand() {
        return this.getCurrentCommand() == this.getDefaultCommand();
    }

    /** Last closed-loop position setpoint this class sent, in rotations. */
    public double getTarget() {
        return target;
    }

    /** Last closed-loop velocity setpoint this class sent, in rotations per second. */
    public double getVelocityTargetRPS() {
        return velocityTarget;
    }

    /** Trigger true while the motor is within tolerance rotations of {@link #getTarget()}. */
    public Trigger atTargetPosition(DoubleSupplier tolerance) {
        return new Trigger(() -> isAtTargetPosition(tolerance));
    }

    /** True while the motor is within tolerance rotations of {@link #getTarget()}. */
    public boolean isAtTargetPosition(DoubleSupplier tolerance) {
        return Math.abs(getPositionRotations() - target) < tolerance.getAsDouble();
    }

    /** Trigger true while the position is within tolerance rotations of target. */
    public Trigger atRotations(DoubleSupplier target, DoubleSupplier tolerance) {
        return near(this::getPositionRotations, target, tolerance);
    }

    /** True while the position is within tolerance rotations of target. */
    public boolean isAtRotations(DoubleSupplier target, DoubleSupplier tolerance) {
        return Math.abs(getPositionRotations() - target.getAsDouble()) < tolerance.getAsDouble();
    }

    /** Trigger true while the position is below target + tolerance, in rotations. */
    public Trigger belowRotations(DoubleSupplier target, DoubleSupplier tolerance) {
        return below(this::getPositionRotations, target, tolerance);
    }

    /** Trigger true while the position is above target - tolerance, in rotations. */
    public Trigger aboveRotations(DoubleSupplier target, DoubleSupplier tolerance) {
        return above(this::getPositionRotations, target, tolerance);
    }

    /**
     * Trigger true while the position is within tolerance percent of target, both of maxRotations.
     */
    public Trigger atPercentage(DoubleSupplier target, DoubleSupplier tolerance) {
        return near(this::getPositionPercentage, target, tolerance);
    }

    /**
     * Trigger true while the position is below target + tolerance, as a percentage of maxRotations.
     */
    public Trigger belowPercentage(DoubleSupplier target, DoubleSupplier tolerance) {
        return below(this::getPositionPercentage, target, tolerance);
    }

    /**
     * Trigger true while the position is above target - tolerance, as a percentage of maxRotations.
     */
    public Trigger abovePercentage(DoubleSupplier target, DoubleSupplier tolerance) {
        return above(this::getPositionPercentage, target, tolerance);
    }

    /** Trigger true while the position is within tolerance degrees of target. */
    public Trigger atDegrees(DoubleSupplier target, DoubleSupplier tolerance) {
        return near(this::getPositionDegrees, target, tolerance);
    }

    /** Trigger true while the position is below target + tolerance, in degrees. */
    public Trigger belowDegrees(DoubleSupplier target, DoubleSupplier tolerance) {
        return below(this::getPositionDegrees, target, tolerance);
    }

    /** Trigger true while the position is above target - tolerance, in degrees. */
    public Trigger aboveDegrees(DoubleSupplier target, DoubleSupplier tolerance) {
        return above(this::getPositionDegrees, target, tolerance);
    }

    /** Trigger true while the velocity is within tolerance RPM of target. */
    public Trigger atVelocityRPM(DoubleSupplier target, DoubleSupplier tolerance) {
        return near(this::getVelocityRPM, target, tolerance);
    }

    /** Trigger true while the velocity is below target + tolerance RPM. */
    public Trigger belowVelocityRPM(DoubleSupplier target, DoubleSupplier tolerance) {
        return below(this::getVelocityRPM, target, tolerance);
    }

    /** Trigger true while the velocity is above target - tolerance RPM. */
    public Trigger aboveVelocityRPM(DoubleSupplier target, DoubleSupplier tolerance) {
        return above(this::getVelocityRPM, target, tolerance);
    }

    /** Trigger true while the stator current is within tolerance amps of target. */
    public Trigger atCurrent(DoubleSupplier target, DoubleSupplier tolerance) {
        return near(this::getStatorCurrent, target, tolerance);
    }

    /** Trigger true while the stator current is below target + tolerance amps. */
    public Trigger belowCurrent(DoubleSupplier target, DoubleSupplier tolerance) {
        return below(this::getStatorCurrent, target, tolerance);
    }

    /** Trigger true while the stator current is above target - tolerance amps. */
    public Trigger aboveCurrent(DoubleSupplier target, DoubleSupplier tolerance) {
        return above(this::getStatorCurrent, target, tolerance);
    }

    /** Active while {@code value} is within {@code tolerance} of {@code target}. */
    private static Trigger near(
            DoubleSupplier value, DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () ->
                        Math.abs(value.getAsDouble() - target.getAsDouble())
                                < tolerance.getAsDouble());
    }

    /** Active while {@code value} is below {@code target + tolerance}. */
    private static Trigger below(
            DoubleSupplier value, DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> value.getAsDouble() < target.getAsDouble() + tolerance.getAsDouble());
    }

    /** Active while {@code value} is above {@code target - tolerance}. */
    private static Trigger above(
            DoubleSupplier value, DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> value.getAsDouble() > target.getAsDouble() - tolerance.getAsDouble());
    }

    /**
     * Refreshes every status signal this mechanism reads, once per robot loop, in one Phoenix call.
     *
     * <p>A Phoenix getter such as {@code motor.getStatorCurrent()} refreshes its own signal by
     * default, and each refresh is a JNI call. Eight signals per mechanism, read once a loop, was
     * over a hundred JNI calls a loop across the robot. One {@code refreshAll} for the lot is
     * CTRE's recommendation, and every read in a loop then sees the same sample.
     */
    private void refreshSignalsOncePerLoop() {
        long loop = RobotLoop.count();
        if (loop == lastSignalRefreshLoop || loopSignals.length == 0) {
            return;
        }
        lastSignalRefreshLoop = loop;
        BaseStatusSignal.refreshAll(loopSignals);
    }

    /** This loop's value of one of the leader's signals, or 0 when there is no hardware. */
    private double signalValue(BaseStatusSignal signal) {
        if (!config.attached) {
            return 0;
        }
        refreshSignalsOncePerLoop();
        return signal.getValueAsDouble();
    }

    /** Motor stator current, in amps, sampled once per loop. */
    public double getStatorCurrent() {
        return signalValue(statorCurrentSignal);
    }

    /** Motor supply current, in amps, sampled once per loop. */
    public double getSupplyCurrent() {
        return signalValue(supplyCurrentSignal);
    }

    /** Applied motor voltage, in volts, sampled once per loop. */
    public double getVoltage() {
        return signalValue(voltageSignal);
    }

    /** Motor temperature in Celsius, sampled once per loop. */
    public double getTemp() {
        return signalValue(tempSignal);
    }

    /**
     * Logs voltage, stator and supply current, temperature and connection under {@code prefix/...}
     * at 10 Hz. Subclasses call this from {@code periodic()}, and the log keys are built on the
     * first call.
     */
    protected void logDiagnostics(String prefix) {
        logDiagnostics(prefix, false);
    }

    /**
     * As {@link #logDiagnostics(String)}, optionally publishing to the dashboard as well.
     *
     * @param dashboard true to publish through {@link Telemetry#logDash}
     */
    protected void logDiagnostics(String prefix, boolean dashboard) {
        if (!prefix.equals(diagnosticsPrefix)) {
            diagnosticsPrefix = prefix;
            voltageKey = prefix + "/Voltage";
            statorCurrentKey = prefix + "/StatorCurrent";
            supplyCurrentKey = prefix + "/SupplyCurrent";
            tempKey = prefix + "/Temp";
            motorConnectedKey = prefix + "/MotorConnected";
        }
        if (!Telemetry.slowLogThisLoop()) {
            // A feedforward fit needs the applied voltage on every velocity sample, not one in
            // five.
            if (config.isFastOutputLogging()) {
                Telemetry.log(voltageKey, getVoltage(), "volts");
            }
            return;
        }
        if (dashboard) {
            Telemetry.logDash(voltageKey, getVoltage(), "volts");
            Telemetry.logDash(statorCurrentKey, getStatorCurrent(), "amps");
            Telemetry.logDash(supplyCurrentKey, getSupplyCurrent(), "amps");
            Telemetry.logDash(tempKey, getTemp(), "deg_C");
            Telemetry.logDash(motorConnectedKey, isMotorConnected());
        } else {
            Telemetry.log(voltageKey, getVoltage(), "volts");
            Telemetry.log(statorCurrentKey, getStatorCurrent(), "amps");
            Telemetry.log(supplyCurrentKey, getSupplyCurrent(), "amps");
            Telemetry.log(tempKey, getTemp(), "deg_C");
            Telemetry.log(motorConnectedKey, isMotorConnected());
        }
    }

    /** How {@link #logStandard} logs {@code <prefix>/RPM}. */
    public enum RpmLog {
        /** Not logged. */
        NONE,
        /** Slow tier, wpilog only. */
        SLOW,
        /** Slow tier, also published to the dashboard. */
        SLOW_DASH,
        /** Every loop, for data that needs it (shot dips, feedforward fits), wpilog only. */
        LOOP,
        /** Every loop, also published to the dashboard. */
        LOOP_DASH
    }

    /**
     * The per-loop logging every mechanism does: battery use, the current command, {@link
     * #logDiagnostics(String, boolean)} and RPM.
     *
     * @param dashboardDiagnostics whether the diagnostics also go to the dashboard
     */
    protected void logStandard(String prefix, boolean dashboardDiagnostics, RpmLog rpm) {
        logBatteryUsage();
        if (!prefix.equals(standardPrefix)) {
            standardPrefix = prefix;
            currentCommandKey = prefix + "/CurrentCommand";
            rpmKey = prefix + "/RPM";
        }
        Telemetry.log(currentCommandKey, getCurrentCommandName());
        logDiagnostics(prefix, dashboardDiagnostics);
        boolean slow = rpm == RpmLog.SLOW || rpm == RpmLog.SLOW_DASH;
        if (rpm == RpmLog.NONE || (slow && !Telemetry.slowLogThisLoop())) {
            return;
        }
        if (rpm == RpmLog.SLOW_DASH || rpm == RpmLog.LOOP_DASH) {
            Telemetry.logDash(rpmKey, getVelocityRPM(), "RPM");
        } else {
            Telemetry.log(rpmKey, getVelocityRPM(), "RPM");
        }
    }

    /** Percentage of {@link Config#getMaxRotations()} to an absolute rotation count. */
    public double percentToRotations(DoubleSupplier percent) {
        return (percent.getAsDouble() / 100) * config.maxRotations;
    }

    /** Absolute rotation count to a percentage of {@link Config#getMaxRotations()}. */
    public double rotationsToPercent(DoubleSupplier rotations) {
        return (rotations.getAsDouble() / config.maxRotations) * 100;
    }

    public double degreesToRotations(DoubleSupplier degrees) {
        return Units.degreesToRotations(degrees.getAsDouble());
    }

    public double rotationsToDegrees(DoubleSupplier rotations) {
        return Units.rotationsToDegrees(rotations.getAsDouble());
    }

    /** Motor position in rotations, sampled once per loop. */
    public double getPositionRotations() {
        return signalValue(positionStatusSignal);
    }

    /**
     * Motor position in rotations, projected from the position signal's own timestamp to now along
     * the velocity signal. A mechanism moving at speed then reads where it is rather than where it
     * was when the CAN frame left the motor. Returns 0 when unattached.
     */
    public double getLatencyCompensatedPositionRotations() {
        if (!config.attached || positionStatusSignal == null) {
            return 0;
        }
        refreshSignalsOncePerLoop();
        return BaseStatusSignal.getLatencyCompensatedValueAsDouble(
                positionStatusSignal, velocityStatusSignal);
    }

    public double getLatencyCompensatedPositionDegrees() {
        return rotationsToDegrees(this::getLatencyCompensatedPositionRotations);
    }

    /** Motor position as a percentage of maxRotations, sampled once per loop. */
    public double getPositionPercentage() {
        return rotationsToPercent(this::getPositionRotations);
    }

    /** Motor position in degrees, sampled once per loop. */
    public double getPositionDegrees() {
        return rotationsToDegrees(this::getPositionRotations);
    }

    /** Motor velocity in RPM, sampled once per loop. */
    public double getVelocityRPM() {
        return Conversions.RPStoRPM(signalValue(velocityStatusSignal));
    }

    /** Runs the mechanism at a constant velocity, in closed-loop voltage control. */
    public Command runVelocity(DoubleSupplier velocityRPM) {
        return run(() -> setVelocity(() -> Conversions.RPMtoRPS(velocityRPM)))
                .withName(getName() + ".runVelocity");
    }

    /** As {@link #runVelocity}, in torque current FOC, which requires Phoenix Pro. */
    public Command runVelocityTcFocRPM(DoubleSupplier velocityRPM) {
        return run(() -> setVelocityTorqueCurrentFOC(() -> Conversions.RPMtoRPS(velocityRPM)))
                .withName(getName() + ".runVelocityTcFocRPM");
    }

    /** Runs the mechanism at an open-loop percent output, in [-1, 1]. */
    public Command runPercentage(DoubleSupplier percent) {
        return run(() -> setPercentOutput(percent)).withName(getName() + ".runPercentage");
    }

    /** Applies a constant voltage in volts, bypassing closed-loop control. */
    public Command runVoltage(DoubleSupplier voltage) {
        return run(() -> setVoltageOutput(voltage)).withName(getName() + ".runVoltage");
    }

    /**
     * Applies a constant voltage in volts and ignores the software limit switches, so it can drive
     * the mechanism past its configured travel.
     */
    public Command runVoltageNoSoftLimit(DoubleSupplier voltage) {
        return run(() -> setVoltageOutputNoSoftLimit(voltage))
                .withName(getName() + ".runVoltageNoSoftLimit");
    }

    /** Holds a torque current in amps through FOC, which requires Phoenix Pro. */
    public Command runTorqueCurrentFoc(DoubleSupplier current) {
        return run(() -> setTorqueCurrentFoc(current)).withName(getName() + ".runTorqueCurrentFoc");
    }

    /**
     * Moves the mechanism to a position in rotations with Motion Magic, in torque current FOC,
     * which requires Phoenix Pro.
     */
    public Command moveToRotations(DoubleSupplier rotations) {
        return run(() -> setMMPositionFoc(rotations)).withName(getName() + ".runPoseRevolutions");
    }

    /** As {@link #moveToRotations}, with the target as a percentage of maxRotations. */
    public Command moveToPercentage(DoubleSupplier percent) {
        return run(() -> setMMPositionFoc(() -> percentToRotations(percent)))
                .withName(getName() + ".runPosePercentage");
    }

    /** As {@link #moveToRotations}, with the target in degrees. */
    public Command moveToDegrees(DoubleSupplier degrees) {
        return run(() -> setMMPositionFoc(() -> degreesToRotations(degrees)))
                .withName(getName() + ".runPoseDegrees");
    }

    /** Same as {@link #moveToRotations}, which is the clearer name. Requires Phoenix Pro. */
    public Command runFocRotations(DoubleSupplier rotations) {
        return run(() -> setMMPositionFoc(rotations)).withName(getName() + ".runFOCPosition");
    }

    public Command runStop() {
        return run(this::stop).withName(getName() + ".runStop");
    }

    /**
     * Coasts while the command runs, then returns to brake. Safe to run while the robot is
     * disabled.
     */
    public Command coastMode() {
        return startEnd(() -> setBrakeMode(false), () -> setBrakeMode(true))
                .ignoringDisable(true)
                .withName(getName() + ".coastMode");
    }

    /** Returns to brake if the mechanism is coasting. Safe to run while the robot is disabled. */
    public Command ensureBrakeMode() {
        return runOnce(() -> setBrakeMode(true))
                .onlyIf(
                        () ->
                                config.attached
                                        && config.talonConfig.MotorOutput.NeutralMode
                                                == NeutralModeValue.Coast)
                .ignoringDisable(true)
                .withName(getName() + ".ensureBrakeMode");
    }

    /** Applies new supply and stator current limits, in amps, to the mechanism. */
    protected Command runCurrentLimits(DoubleSupplier supplyLimit, DoubleSupplier statorLimit) {
        return Commands.runOnce(() -> setCurrentLimits(supplyLimit, statorLimit));
    }

    protected void setCurrentLimits(DoubleSupplier supplyLimit, DoubleSupplier statorLimit) {
        applyCurrentLimit(supplyLimit, statorLimit);
    }

    /** Stops the motor output. Does nothing if the mechanism is not attached. */
    public void stop() {
        if (isAttached()) {
            motor.stopMotor();
        }
    }

    /**
     * Sets the mechanism's reported position to zero (tares the motor encoder). Does nothing if the
     * mechanism is not attached.
     */
    protected void tareMotor() {
        if (isAttached()) {
            setMotorPosition(() -> 0);
        }
    }

    /** Writes the motor's internal position register, in rotations, without moving the motor. */
    protected void setMotorPosition(DoubleSupplier rotations) {
        if (isAttached()) {
            motor.setPosition(rotations.getAsDouble());
        }
    }

    /** Velocity in rotations per second, via Motion Magic in torque current FOC (Phoenix Pro). */
    protected void setMMVelocityFOC(DoubleSupplier velocityRPS) {
        if (isAttached()) {
            velocityTarget = velocityRPS.getAsDouble();
            MotionMagicVelocityTorqueCurrentFOC mm =
                    config.mmVelocityFOC.withVelocity(velocityTarget);
            motor.setControl(mm);
        }
    }

    /** Velocity in rotations per second, in torque current FOC (requires Phoenix Pro). */
    protected void setVelocityTorqueCurrentFOC(DoubleSupplier velocityRPS) {
        if (isAttached()) {
            velocityTarget = velocityRPS.getAsDouble();
            VelocityTorqueCurrentFOC output =
                    config.velocityTorqueCurrentFOC.withVelocity(velocityTarget);
            motor.setControl(output);
        }
    }

    /** As {@link #setVelocityTorqueCurrentFOC}, taking RPM. Requires Phoenix Pro. */
    protected void setVelocityTCFOCrpm(DoubleSupplier velocityRPM) {
        setVelocityTorqueCurrentFOC(() -> Conversions.RPMtoRPS(velocityRPM.getAsDouble()));
    }

    /** Velocity in rotations per second, in closed-loop voltage control. */
    protected void setVelocity(DoubleSupplier velocityRPS) {
        if (isAttached()) {
            velocityTarget = velocityRPS.getAsDouble();
            VelocityVoltage output = config.velocityControl.withVelocity(velocityTarget);
            motor.setControl(output);
        }
    }

    /** As {@link #setVelocity}, taking RPM. */
    protected void setVelocityRPM(DoubleSupplier velocityRPM) {
        setVelocity(() -> Conversions.RPMtoRPS(velocityRPM.getAsDouble()));
    }

    /** Position control with voltage compensation and no velocity feedforward. */
    protected void setPosition(DoubleSupplier rotations) {
        setPositionWithVelocity(rotations, () -> 0);
    }

    /**
     * Position control with an explicit velocity feedforward. This is the right primitive for a
     * continuously moving setpoint, such as a turret tracking a target while the robot drives,
     * where a profiled motion would leave steady-state lag.
     *
     * @param rotations the target position in mechanism rotations
     * @param velocityRPS the feedforward in mechanism rotations per second
     */
    protected void setPositionWithVelocity(DoubleSupplier rotations, DoubleSupplier velocityRPS) {
        if (isAttached()) {
            target = rotations.getAsDouble();
            PositionVoltage output =
                    config.positionControl
                            .withPosition(target)
                            .withVelocity(velocityRPS.getAsDouble());
            motor.setControl(output);
        }
    }

    /** Position in rotations, via Motion Magic in torque current FOC (requires Phoenix Pro). */
    protected void setMMPositionFoc(DoubleSupplier rotations) {
        if (isAttached()) {
            target = rotations.getAsDouble();
            MotionMagicTorqueCurrentFOC mm = config.mmPositionFOC.withPosition(target);
            motor.setControl(mm);
        }
    }

    /**
     * As {@link #setPositionWithVelocity}, in {@link PositionTorqueCurrentFOC}, which requires
     * Phoenix Pro.
     *
     * @param rotations the target position in mechanism rotations
     * @param velocityRPS the feedforward in mechanism rotations per second
     */
    protected void setPositionFocWithVelocity(
            DoubleSupplier rotations, DoubleSupplier velocityRPS) {
        if (isAttached()) {
            target = rotations.getAsDouble();
            PositionTorqueCurrentFOC req =
                    config.positionTorqueFOC
                            .withPosition(target)
                            .withVelocity(velocityRPS.getAsDouble());
            motor.setControl(req);
        }
    }

    /**
     * Dynamic Motion Magic in torque current FOC, which requires Phoenix Pro. The trajectory
     * parameters can change every loop cycle.
     *
     * @param velocity cruise velocity in rotations per second
     * @param acceleration in rotations per second squared
     * @param jerk in rotations per second cubed
     */
    protected void setDynMMPositionFoc(
            DoubleSupplier rotations,
            DoubleSupplier velocity,
            DoubleSupplier acceleration,
            DoubleSupplier jerk) {
        if (isAttached()) {
            target = rotations.getAsDouble();
            DynamicMotionMagicTorqueCurrentFOC mm =
                    config.dynamicMMPositionFOC
                            .withPosition(target)
                            .withVelocity(velocity.getAsDouble())
                            .withAcceleration(acceleration.getAsDouble())
                            .withJerk(jerk.getAsDouble());
            motor.setControl(mm);
        }
    }

    /**
     * Dynamic Motion Magic with voltage compensation. The trajectory parameters can change every
     * loop cycle.
     *
     * @param velocity cruise velocity in rotations per second
     * @param acceleration in rotations per second squared
     * @param jerk in rotations per second cubed
     */
    protected void setDynMMPositionVoltage(
            DoubleSupplier rotations,
            DoubleSupplier velocity,
            DoubleSupplier acceleration,
            DoubleSupplier jerk) {
        if (isAttached()) {
            target = rotations.getAsDouble();
            DynamicMotionMagicVoltage mm =
                    config.dynamicMotionMagicVoltage
                            .withPosition(target)
                            .withVelocity(velocity.getAsDouble())
                            .withAcceleration(acceleration.getAsDouble())
                            .withJerk(jerk.getAsDouble());
            motor.setControl(mm);
        }
    }

    /** Position in rotations, via Motion Magic in voltage control on slot 0. */
    protected void setMMPosition(DoubleSupplier rotations) {
        setMMPosition(rotations, 0);
    }

    /**
     * Position in rotations, via Motion Magic in voltage control on a chosen gain slot.
     *
     * @param slot the gain slot, 0, 1 or 2
     */
    protected void setMMPosition(DoubleSupplier rotations, int slot) {
        if (isAttached()) {
            target = rotations.getAsDouble();
            MotionMagicVoltage mm =
                    config.mmPositionVoltageSlot.withSlot(slot).withPosition(target);
            motor.setControl(mm);
        }
    }

    /**
     * Open-loop percent output, in [-1, 1]. The applied voltage is percent times {@link
     * Config#getVoltageCompSaturation()}.
     */
    protected void setPercentOutput(DoubleSupplier percent) {
        setVoltageOutput(() -> config.voltageCompSaturation * percent.getAsDouble());
    }

    /** Open-loop voltage control. Applies the requested volts with no scaling. */
    protected void setVoltageOutput(DoubleSupplier voltage) {
        voltageOut(voltage, false);
    }

    /**
     * Open-loop voltage control that ignores the software limit switches, so it can drive the
     * mechanism past its configured travel.
     */
    protected void setVoltageOutputNoSoftLimit(DoubleSupplier voltage) {
        voltageOut(voltage, true);
    }

    /**
     * Sends the shared voltage request. The soft-limit flag is set on every call because the
     * request object is reused.
     */
    private void voltageOut(DoubleSupplier voltage, boolean ignoreSoftLimits) {
        if (isAttached()) {
            VoltageOut output =
                    config.voltageControl
                            .withOutput(voltage.getAsDouble())
                            .withIgnoreSoftwareLimits(ignoreSoftLimits);
            motor.setControl(output);
        }
    }

    /** Torque current in amps through FOC, which requires Phoenix Pro. */
    public void setTorqueCurrentFoc(DoubleSupplier current) {
        if (isAttached()) {
            TorqueCurrentFOC output = config.torqueCurrentFOC.withOutput(current.getAsDouble());
            motor.setControl(output);
        }
    }

    /** Sets brake or coast mode and applies it to the hardware immediately. */
    public void setBrakeMode(boolean isInBrake) {
        if (isAttached()) {
            config.configNeutralBrakeMode(isInBrake);
            config.applyTalonConfig(motor);
        }
    }

    /**
     * Enables or disables the reverse software limit and applies the change immediately. The
     * threshold comes from the current configuration.
     */
    public void toggleReverseSoftLimit(boolean enabled) {
        if (isAttached()) {
            double threshold = config.talonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold;
            config.configReverseSoftLimit(threshold, enabled);
            config.applyTalonConfig(motor);
        }
    }

    /**
     * Enables or disables a forward and reverse torque current limit and applies the change
     * immediately. Disabling resets the peak to plus or minus 300 A, which is effectively
     * unlimited.
     *
     * @param enabledLimit the torque current limit in amps, applied when enabled is true
     */
    public void toggleTorqueCurrentLimit(DoubleSupplier enabledLimit, boolean enabled) {
        if (isAttached()) {
            if (enabled) {
                config.configForwardTorqueCurrentLimit(enabledLimit.getAsDouble());
                config.configReverseTorqueCurrentLimit(-1 * enabledLimit.getAsDouble());
                config.configStatorCurrentLimit(enabledLimit.getAsDouble(), true);
            } else {
                config.configForwardTorqueCurrentLimit(300);
                config.configReverseTorqueCurrentLimit(-300);
            }
            config.applyTalonConfig(motor);
        }
    }

    /**
     * Enables or disables the supply current limit and applies the change immediately.
     *
     * @param enabledLimit the supply current limit in amps
     */
    public void toggleSupplyCurrentLimit(DoubleSupplier enabledLimit, boolean enabled) {
        if (isAttached()) {
            config.configSupplyCurrentLimit(enabledLimit.getAsDouble(), enabled);
            config.applyTalonConfig(motor);
        }
    }

    /**
     * Applies new supply and stator current limits, in amps, but only when they differ from the
     * configured ones. The apply is retried while {@link CanConfigBudget#maxAttempts()} allows it.
     */
    public void applyCurrentLimit(DoubleSupplier supplyLimit, DoubleSupplier statorLimit) {
        if (isAttached()) {
            if (config.talonConfig.CurrentLimits.StatorCurrentLimit != statorLimit.getAsDouble()
                    || config.talonConfig.CurrentLimits.SupplyCurrentLimit
                            != supplyLimit.getAsDouble()) {
                config.configSupplyCurrentLimit(Math.abs(supplyLimit.getAsDouble()), true);
                config.configStatorCurrentLimit(Math.abs(statorLimit.getAsDouble()), true);
                config.configForwardTorqueCurrentLimit(Math.abs(statorLimit.getAsDouble()));
                config.configReverseTorqueCurrentLimit(-1 * Math.abs(statorLimit.getAsDouble()));
                int attempts = CanConfigBudget.maxAttempts();
                for (int i = 0; i < attempts; i++) {
                    StatusCode result =
                            CanConfigBudget.run(
                                    config.getName(),
                                    timeout ->
                                            motor.getConfigurator()
                                                    .apply(config.talonConfig, timeout));
                    if (!result.isOK()) {
                        System.out.println(
                                "Could not apply config changes to "
                                        + config.getName()
                                        + "\'s motor ");
                    } else {
                        break;
                    }
                }
            }
        }
    }

    /**
     * Averages the stator current over the command's runtime and raises a warning {@link Alert} if
     * the average is further than tolerance from expectedCurrent.
     *
     * @param expectedCurrent the expected average stator current in amps
     * @param tolerance the largest acceptable deviation in amps
     */
    public Command checkAvgCurrent(DoubleSupplier expectedCurrent, DoubleSupplier tolerance) {
        return new Command() {
            double totalCurrent = 0;
            int count = 0;
            String alertText = config.name + " AvgCurrent Error";

            @Override
            public void initialize() {
                totalCurrent = 0;
                count = 0;
            }

            @Override
            public void execute() {
                totalCurrent += getStatorCurrent();
                count++;
            }

            @Override
            public void end(boolean interrupted) {
                double avgCurrent = totalCurrent / count;
                if (Math.abs(avgCurrent - expectedCurrent.getAsDouble())
                        > tolerance.getAsDouble()) {
                    currentAlert.setText(
                            alertText
                                    + " Expected: "
                                    + expectedCurrent.getAsDouble()
                                    + " Actual: "
                                    + avgCurrent);
                    currentAlert.set(true);
                }
            }
        };
    }

    /**
     * Tracks the peak stator current and raises a warning {@link Alert} if it exceeds
     * expectedCurrent.
     *
     * @param expectedCurrent the highest acceptable peak stator current in amps
     */
    public Command checkMaxCurrent(DoubleSupplier expectedCurrent) {
        return checkPeakCurrent(expectedCurrent, true, " MaxCurrent Error Expected: ");
    }

    /**
     * Tracks the peak stator current and raises a warning {@link Alert} if it never reaches
     * expectedCurrent, which checks that a mechanism drew at least its expected load.
     *
     * @param expectedCurrent the lowest acceptable peak stator current in amps
     */
    public Command checkMinThresholdCurrent(DoubleSupplier expectedCurrent) {
        return checkPeakCurrent(expectedCurrent, false, " Current Error Expected at least: ");
    }

    /**
     * Tracks the peak stator current and raises {@link #currentAlert} if it lands on the wrong side
     * of expected.
     */
    private Command checkPeakCurrent(
            DoubleSupplier expectedCurrent, boolean failAbove, String message) {
        return new Command() {
            double maxCurrent = 0;

            @Override
            public void initialize() {
                maxCurrent = 0;
            }

            @Override
            public void execute() {
                maxCurrent = Math.max(maxCurrent, getStatorCurrent());
            }

            @Override
            public void end(boolean interrupted) {
                double expected = expectedCurrent.getAsDouble();
                if (failAbove ? maxCurrent > expected : maxCurrent < expected) {
                    currentAlert.setText(
                            config.name + message + expected + " Actual: " + maxCurrent);
                    currentAlert.set(true);
                }
            }
        };
    }

    /**
     * A TalonFX follower that mirrors the leader's output. Set {@code opposeLeader} to {@link
     * MotorAlignmentValue#Opposed} when the motor is mounted in the opposite direction and has to
     * spin in reverse to move the mechanism the same way.
     */
    public static class FollowerConfig {

        /** Name for this follower, used in its alerts and log keys. */
        @Getter private String name;

        @Getter private CanDeviceId id;

        @Getter private boolean attached = true;

        /**
         * Alignment of the follower relative to the leader. Use {@link MotorAlignmentValue#Opposed}
         * when the follower is physically mounted in the opposite direction.
         */
        @Getter private MotorAlignmentValue opposeLeader = MotorAlignmentValue.Aligned;

        /**
         * @param canbus CAN bus name, such as {@code "rio"} or {@code "canivore"}
         */
        public FollowerConfig(
                String name, int id, String canbus, MotorAlignmentValue opposeLeader) {
            this.name = name;
            this.id = new CanDeviceId(id, canbus);
            this.opposeLeader = opposeLeader;
        }
    }

    /**
     * TalonFX hardware configuration, the control request objects, and the mechanism-level
     * parameters (gear ratio, soft limits, PID/FF gains, Motion Magic profile, current limits).
     *
     * <p>Subclass this per mechanism, call the {@code config*()} helpers in the subclass
     * constructor, and pass the result to the {@link Mechanism} constructor.
     */
    public static class Config {

        /** Name for this mechanism, used in log keys and alerts. */
        @Getter private String name;

        /** False builds no hardware, which is how the sim runs a real mechanism config. */
        @Getter @Setter private boolean attached = true;

        /**
         * Publish the leader's output signals (duty cycle, motor voltage, torque current) at the
         * control rate and log its applied voltage on every loop instead of at 10 Hz. Set it on the
         * mechanisms whose feedforward is fit from logs: kS and kV survive a 10 Hz voltage, kA does
         * not, because the acceleration it multiplies is gone between one sample and the next.
         * Costs one status frame at 100 Hz on this motor and one double per loop in the log; the
         * followers are untouched.
         */
        @Getter @Setter private boolean fastOutputLogging = false;

        @Getter private CanDeviceId id;

        /** Applied to the leader motor on startup. */
        @Getter @Setter protected TalonFXConfiguration talonConfig;

        /** Leader plus followers. */
        @Getter private int numMotors = 1;

        /** Saturation for {@link Mechanism#setPercentOutput}, 12 V by default. */
        @Getter private double voltageCompSaturation = 12.0;

        /** Bottom of the mechanism's range, in rotations, for the percentage helpers. */
        @Getter private double minRotations = 0;

        /** Top of the mechanism's range, in rotations, for the percentage helpers. */
        @Getter private double maxRotations = 1;

        /** Empty when the mechanism runs on the leader alone. */
        @Getter private FollowerConfig[] followerConfigs = new FollowerConfig[0];

        // Pre-built control requests, reused each loop to keep the loop allocation free.

        @Getter
        private MotionMagicVelocityTorqueCurrentFOC mmVelocityFOC =
                new MotionMagicVelocityTorqueCurrentFOC(0);

        @Getter
        private MotionMagicTorqueCurrentFOC mmPositionFOC = new MotionMagicTorqueCurrentFOC(0);

        @Getter
        private PositionTorqueCurrentFOC positionTorqueFOC = new PositionTorqueCurrentFOC(0);

        @Getter
        private DynamicMotionMagicTorqueCurrentFOC dynamicMMPositionFOC =
                new DynamicMotionMagicTorqueCurrentFOC(0, 0, 0);

        @Getter
        private DynamicMotionMagicVoltage dynamicMotionMagicVoltage =
                new DynamicMotionMagicVoltage(0, 0, 0);

        @Getter
        private MotionMagicVelocityVoltage mmVelocityVoltage = new MotionMagicVelocityVoltage(0);

        @Getter private MotionMagicVoltage mmPositionVoltage = new MotionMagicVoltage(0);

        @Getter
        private MotionMagicVoltage mmPositionVoltageSlot = new MotionMagicVoltage(0).withSlot(1);

        @Getter private VoltageOut voltageControl = new VoltageOut(0);
        @Getter private VelocityVoltage velocityControl = new VelocityVoltage(0);
        @Getter private PositionVoltage positionControl = new PositionVoltage(0);

        @Getter
        private VelocityTorqueCurrentFOC velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);

        @Getter private TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(0);

        /** Percent output control. Prefer {@link #voltageControl} in most cases. */
        @Getter private DutyCycleOut percentOutput = new DutyCycleOut(0);

        /**
         * Leaves the Talon settings at their defaults, with the hardware limit switches disabled.
         *
         * @param canbus CAN bus name, such as {@code "rio"} or {@code "canivore"}
         */
        public Config(String name, int id, String canbus) {
            this.name = name;
            this.id = new CanDeviceId(id, canbus);
            talonConfig = new TalonFXConfiguration();

            talonConfig.HardwareLimitSwitch.ForwardLimitEnable = false;
            talonConfig.HardwareLimitSwitch.ReverseLimitEnable = false;
        }

        /**
         * Applies the current {@link TalonFXConfiguration} to the given motor, reporting a Driver
         * Station warning if the apply fails.
         */
        public void applyTalonConfig(TalonFX talon) {
            StatusCode result =
                    CanConfigBudget.run(
                            name, timeout -> talon.getConfigurator().apply(talonConfig, timeout));
            if (!result.isOK()) {
                DriverStation.reportWarning(
                        "Could not apply config changes to " + name + "\'s motor ", false);
            }
        }

        public void setFollowerConfigs(FollowerConfig... followers) {
            followerConfigs = followers;
        }

        public void configVoltageCompensation(double voltageCompSaturation) {
            this.voltageCompSaturation = voltageCompSaturation;
        }

        /**
         * Counter-clockwise positive, which is the right default for most mechanisms when read from
         * the shaft end.
         */
        public void configCounterClockwise_Positive() {
            talonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        }

        public void configClockwise_Positive() {
            talonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        }

        /** Sets the peak forward output voltage, in volts. */
        public void configForwardVoltageLimit(double voltageLimit) {
            talonConfig.Voltage.PeakForwardVoltage = voltageLimit;
        }

        /** Sets the peak reverse output voltage, in volts. */
        public void configReverseVoltageLimit(double voltageLimit) {
            talonConfig.Voltage.PeakReverseVoltage = voltageLimit;
        }

        /**
         * Sets the current limits as a group: the supply limit with its lower limit and lower-limit
         * time, the stator limit, and both torque limits at the stator value.
         *
         * @param statorLimit stator current limit in amps, also used for both torque limits
         */
        public void configCurrentLimits(
                double supplyLimit,
                double statorLimit,
                double lowerSupplyLimit,
                double lowerSupplyTime) {
            configSupplyCurrentLimit(supplyLimit, true);
            configStatorCurrentLimit(statorLimit, true);
            configLowerSupplyCurrentLimit(lowerSupplyLimit);
            configLowerSupplyCurrentTime(lowerSupplyTime);
            configForwardTorqueCurrentLimit(statorLimit);
            configReverseTorqueCurrentLimit(statorLimit);
        }

        /**
         * Configures the supply current limit, in amps, from the absolute value of the argument, so
         * a negative limit is corrected rather than rejected.
         */
        public void configSupplyCurrentLimit(double supplyLimit, boolean enabled) {
            talonConfig.CurrentLimits.SupplyCurrentLimit = Math.abs(supplyLimit);
            talonConfig.CurrentLimits.SupplyCurrentLimitEnable = enabled;
        }

        /**
         * Configures the stator current limit, in amps, from the absolute value of the argument, so
         * a negative limit is corrected rather than rejected.
         */
        public void configStatorCurrentLimit(double statorLimit, boolean enabled) {
            talonConfig.CurrentLimits.StatorCurrentLimit = Math.abs(statorLimit);
            talonConfig.CurrentLimits.StatorCurrentLimitEnable = enabled;
        }

        /**
         * Sets the peak forward torque current limit, in amps, from the absolute value of the
         * argument, so a negative limit is corrected rather than rejected.
         */
        public void configForwardTorqueCurrentLimit(double currentLimit) {
            talonConfig.TorqueCurrent.PeakForwardTorqueCurrent = Math.abs(currentLimit);
        }

        /** Sets the open-loop ramp period, in seconds, for duty cycle, voltage and torque. */
        public void configOpenLoopRamps(double seconds) {
            talonConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = seconds;
            talonConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = seconds;
            talonConfig.OpenLoopRamps.TorqueOpenLoopRampPeriod = seconds;
        }

        /** Sets the closed-loop ramp period, in seconds, for duty cycle, voltage and torque. */
        public void configClosedLoopRamps(double seconds) {
            talonConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = seconds;
            talonConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = seconds;
            talonConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = seconds;
        }

        /**
         * Sets the peak reverse torque current limit, in amps. The stored value is forced negative,
         * so a positive argument is corrected rather than rejected.
         */
        public void configReverseTorqueCurrentLimit(double currentLimit) {
            talonConfig.TorqueCurrent.PeakReverseTorqueCurrent = -Math.abs(currentLimit);
        }

        /** Sets the lower supply current limit, in amps, that applies once the upper one trips. */
        public void configLowerSupplyCurrentLimit(double currentLimit) {
            talonConfig.CurrentLimits.SupplyCurrentLowerLimit = currentLimit;
        }

        /** Sets how long the upper supply limit holds, in seconds, before dropping to the lower. */
        public void configLowerSupplyCurrentTime(double time) {
            talonConfig.CurrentLimits.SupplyCurrentLowerTime = time;
        }

        /**
         * Sets the duty-cycle neutral deadband as a fraction of full output, so outputs below that
         * magnitude are treated as zero.
         */
        public void configNeutralDeadband(double deadband) {
            talonConfig.MotorOutput.DutyCycleNeutralDeadband = deadband;
        }

        /**
         * Sets the peak duty-cycle output limits.
         *
         * @param forward maximum forward output, [0, 1]
         * @param reverse maximum reverse output, [-1, 0]
         */
        public void configPeakOutput(double forward, double reverse) {
            talonConfig.MotorOutput.PeakForwardDutyCycle = forward;
            talonConfig.MotorOutput.PeakReverseDutyCycle = reverse;
        }

        /** Configures the forward software limit, with the threshold in rotations. */
        public void configForwardSoftLimit(double threshold, boolean enabled) {
            talonConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = threshold;
            talonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = enabled;
        }

        /** Configures the reverse software limit, with the threshold in rotations. */
        public void configReverseSoftLimit(double threshold, boolean enabled) {
            talonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = threshold;
            talonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = enabled;
        }

        /**
         * Enables or disables continuous wrap-around in closed-loop control, for a mechanism that
         * turns indefinitely such as a swerve azimuth.
         */
        public void configContinuousWrap(boolean enabled) {
            talonConfig.ClosedLoopGeneral.ContinuousWrap = enabled;
        }

        /**
         * Sets the Motion Magic acceleration and feed-forward on both the FOC and the voltage
         * velocity requests.
         *
         * @param acceleration in rotations per second squared
         */
        public void configMotionMagicVelocity(double acceleration, double feedforward) {
            mmVelocityFOC =
                    mmVelocityFOC.withAcceleration(acceleration).withFeedForward(feedforward);
            mmVelocityVoltage =
                    mmVelocityVoltage.withAcceleration(acceleration).withFeedForward(feedforward);
        }

        /** Sets the feed-forward term on every Motion Magic position request. */
        public void configMotionMagicPosition(double feedforward) {
            mmPositionFOC = mmPositionFOC.withFeedForward(feedforward);
            mmPositionVoltage = mmPositionVoltage.withFeedForward(feedforward);
            mmPositionVoltageSlot = mmPositionVoltageSlot.withFeedForward(feedforward);
        }

        /**
         * Sets the Motion Magic trajectory limits.
         *
         * @param cruiseVelocity in rotations per second
         * @param acceleration in rotations per second squared
         * @param jerk in rotations per second cubed
         */
        public void configMotionMagic(double cruiseVelocity, double acceleration, double jerk) {
            talonConfig.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity;
            talonConfig.MotionMagic.MotionMagicAcceleration = acceleration;
            talonConfig.MotionMagic.MotionMagicJerk = jerk;
        }

        /**
         * Sets the sensor-to-mechanism gear ratio, counted in sensor turns per mechanism output
         * turn, so 11.25 means 11.25 sensor turns per output rotation.
         */
        public void configGearRatio(double gearRatio) {
            talonConfig.Feedback.SensorToMechanismRatio = gearRatio;
        }

        /** The configured sensor-to-mechanism gear ratio. */
        public double getGearRatio() {
            return talonConfig.Feedback.SensorToMechanismRatio;
        }

        /** Sets brake mode when isInBrake is true, coast when it is false. */
        public void configNeutralBrakeMode(boolean isInBrake) {
            if (isInBrake) {
                talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            } else {
                talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            }
        }

        /** Configures the PID gains in slot 0. */
        public void configPIDGains(double kP, double kI, double kD) {
            configPIDGains(0, kP, kI, kD);
        }

        /**
         * Configures the PID gains in a gain slot.
         *
         * @param slot the gain slot, 0, 1 or 2
         */
        public void configPIDGains(int slot, double kP, double kI, double kD) {
            switch (slot) {
                case 0 -> talonConfig.Slot0.withKP(kP).withKI(kI).withKD(kD);
                case 1 -> talonConfig.Slot1.withKP(kP).withKI(kI).withKD(kD);
                case 2 -> talonConfig.Slot2.withKP(kP).withKI(kI).withKD(kD);
                default -> DriverStation.reportWarning("MechConfig: Invalid Feedback slot", false);
            }
        }

        /**
         * Configures the feed-forward gains in slot 0.
         *
         * @param kS static friction compensation, in volts or amps depending on the control mode
         */
        public void configFeedForwardGains(double kS, double kV, double kA, double kG) {
            configFeedForwardGains(0, kS, kV, kA, kG);
        }

        /**
         * Configures the feed-forward gains in a gain slot.
         *
         * @param slot the gain slot, 0, 1 or 2
         * @param kS static friction compensation, in volts or amps depending on the control mode
         */
        public void configFeedForwardGains(int slot, double kS, double kV, double kA, double kG) {
            switch (slot) {
                case 0 -> talonConfig.Slot0.withKS(kS).withKV(kV).withKA(kA).withKG(kG);
                case 1 -> talonConfig.Slot1.withKS(kS).withKV(kV).withKA(kA).withKG(kG);
                case 2 -> talonConfig.Slot2.withKS(kS).withKV(kV).withKA(kA).withKG(kG);
                default -> DriverStation.reportWarning(
                        "MechConfig: Invalid FeedForward slot", false);
            }
        }

        /** Configures the feedback sensor source with a rotor offset of 0. */
        public void configFeedbackSensorSource(FeedbackSensorSourceValue source) {
            configFeedbackSensorSource(source, 0);
        }

        /** Configures the feedback sensor source and its rotor offset, in rotations. */
        public void configFeedbackSensorSource(FeedbackSensorSourceValue source, double offset) {
            talonConfig.Feedback.FeedbackSensorSource = source;
            talonConfig.Feedback.FeedbackRotorOffset = offset;
        }

        /**
         * Configures the gravity compensation type in slot 0.
         *
         * @param isArm true for {@link GravityTypeValue#Arm_Cosine} on a rotating arm, false for
         *     {@link GravityTypeValue#Elevator_Static} on an elevator
         */
        public void configGravityType(boolean isArm) {
            configGravityType(0, isArm);
        }

        /**
         * Configures the gravity compensation type in a gain slot.
         *
         * @param slot the gain slot, 0, 1 or 2
         * @param isArm true for {@link GravityTypeValue#Arm_Cosine} on a rotating arm, false for
         *     {@link GravityTypeValue#Elevator_Static} on an elevator
         */
        public void configGravityType(int slot, boolean isArm) {
            GravityTypeValue gravityType =
                    isArm ? GravityTypeValue.Arm_Cosine : GravityTypeValue.Elevator_Static;
            switch (slot) {
                case 0 -> talonConfig.Slot0.GravityType = gravityType;
                case 1 -> talonConfig.Slot1.GravityType = gravityType;
                case 2 -> talonConfig.Slot2.GravityType = gravityType;
                default -> DriverStation.reportWarning("MechConfig: Invalid slot", false);
            }
        }

        /**
         * Sets the rotation limits the percentage helpers such as {@link
         * Mechanism#percentToRotations} convert against.
         */
        protected void configMinMaxRotations(double minRotation, double maxRotation) {
            this.minRotations = minRotation;
            this.maxRotations = maxRotation;
        }
    }
}
