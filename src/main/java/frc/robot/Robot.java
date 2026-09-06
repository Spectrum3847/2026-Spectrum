package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.rebuilt.ShiftHelpers;
import frc.rebuilt.ShotCalculator;
import frc.rebuilt.targetFactories.FeedTargetFactory;
import frc.robot.auton.Auton;
import frc.robot.configs.OM2026;
import frc.robot.operator.Operator;
import frc.robot.operator.Operator.OperatorConfig;
import frc.robot.pilot.Pilot;
import frc.robot.pilot.Pilot.PilotConfig;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.WantedSuperState;
import frc.robot.subsystems.dyeRotor.DyeRotor;
import frc.robot.subsystems.dyeRotor.DyeRotor.DyeRotorConfig;
import frc.robot.subsystems.dyeRotor.DyeRotor.Feeder.FeederConfig;
import frc.robot.subsystems.dyeRotor.DyeRotor.Rotor.RotorConfig;
import frc.robot.subsystems.fuelIntake.FuelIntake;
import frc.robot.subsystems.fuelIntake.FuelIntake.FuelIntakeConfig;
import frc.robot.subsystems.fuelIntake.FuelIntake.IntakeKicker.IntakeKickerConfig;
import frc.robot.subsystems.fuelIntake.FuelIntake.IntakeRoller.IntakeRollerConfig;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.Hood.HoodConfig;
import frc.robot.subsystems.intakeExtension.IntakeExtension;
import frc.robot.subsystems.intakeExtension.IntakeExtension.IntakeExtensionConfig;
import frc.robot.subsystems.intakeExtension.IntakeExtension.Left.LeftConfig;
import frc.robot.subsystems.intakeExtension.IntakeExtension.Right.RightConfig;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.Launcher.LauncherConfig;
import frc.robot.subsystems.launcher.LauncherTower;
import frc.robot.subsystems.launcher.LauncherTower.LauncherTowerConfig;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConfig;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.Turret.TurretConfig;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.Vision.VisionConfig;
import frc.spectrumLib.framework.RobotLoop;
import frc.spectrumLib.framework.SpectrumRobot;
import frc.spectrumLib.hardware.Rio;
import frc.spectrumLib.telemetry.BatteryLogger;
import frc.spectrumLib.telemetry.SystemLoadMonitor;
import frc.spectrumLib.telemetry.Telemetry;
import frc.spectrumLib.telemetry.Telemetry.PrintPriority;
import frc.spectrumLib.util.CrashTracker;
import frc.spectrumLib.util.Util;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;
import lombok.Getter;
import org.json.simple.parser.ParseException;

/**
 * The main robot class. This class is the entry point for the robot code and manages all subsystems
 * and their configurations. The main robot class. This class is the entry point for the robot code
 * and manages all subsystems and their configurations.
 */
public class Robot extends SpectrumRobot {
    @Getter private static RobotSim robotSim;
    @Getter private static Config config;
    @Getter private static final Field2d field2d = new Field2d();
    public static Telemetry telemetry = new Telemetry();
    public static boolean autonWarmedUp = false;

    /** CPU, loop period, GC and memory on the dashboard, with alerts when they stay bad. */
    private final SystemLoadMonitor systemLoad = new SystemLoadMonitor();

    public static class Config {
        public final SwerveConfig swerve = new SwerveConfig();
        public final PilotConfig pilot = new PilotConfig();
        public final OperatorConfig operator = new OperatorConfig();
        public final IntakeRollerConfig intakeRoller = new IntakeRollerConfig();
        public final IntakeKickerConfig intakeKicker = new IntakeKickerConfig();
        public final FuelIntakeConfig fuelIntake = new FuelIntakeConfig(intakeRoller, intakeKicker);
        public final LeftConfig intakeExtensionLeft = new LeftConfig();
        public final RightConfig intakeExtensionRight = new RightConfig(intakeExtensionLeft);
        public final IntakeExtensionConfig intakeExtension =
                new IntakeExtensionConfig(intakeExtensionLeft, intakeExtensionRight);
        public final RotorConfig rotor = new RotorConfig();
        public final FeederConfig feeder = new FeederConfig();
        public final DyeRotorConfig dyeRotor = new DyeRotorConfig(rotor, feeder);
        public final LauncherConfig launcher = new LauncherConfig();
        public final VisionConfig vision = new VisionConfig();
        public final TurretConfig turret = new TurretConfig();
        public final HoodConfig hood = new HoodConfig();
        public final LauncherTowerConfig launcherTower = new LauncherTowerConfig();
    }

    @Getter private static Swerve swerve;
    @Getter private static FuelIntake fuelIntake;
    @Getter private static IntakeExtension intakeExtension;
    @Getter private static DyeRotor dyeRotor;
    @Getter private static Operator operator;
    @Getter private static Pilot pilot;
    @Getter private static Turret turret;
    @Getter private static Launcher launcher;
    @Getter private static LauncherTower launcherTower;
    @Getter private static Hood hood;
    @Getter private static Vision vision;
    // @Getter private static Leds leds;
    @Getter private static Auton auton;

    @Getter private static SuperStructure superStructure;
    @Getter private static BatteryLogger batteryLogger;
    @Getter private static CANBus mainCANBus;

    /** Creates a new Robot instance. */
    public Robot() {
        super();
        /*
         * Phoenix otherwise starts writing .hoot signal logs for every CAN device a second after
         * the first enable. Nobody replays them in Tuner X, and on 2026-09-05 they were a large
         * share of the 2.2 GB on the rio's SD card, written alongside the wpilog on a machine whose
         * CPU was already at 92-95%. SignalLogger.start() still works for a deliberate capture.
         */
        SignalLogger.enableAutoLogging(false);

        // Mirror-to-NetworkTables off; see Telemetry.start() for what the dashboard gets instead.
        Telemetry.start(false, true, false, true, false, true, PrintPriority.NORMAL);

        try {
            Telemetry.print("--- Robot Init Starting ---");

            // Set up the config
            switch (Rio.id) {
                default:
                    config = new OM2026();
                    break;
            }

            double canInitDelay = 0.1; // Delay between any mechanism with motor/can configs
            mainCANBus = new CANBus(Rio.CANIVORE); // Use the first CANivore bus found

            pilot = new Pilot(config.pilot);
            operator = new Operator(config.operator);

            swerve = new Swerve(config.swerve);
            Timer.delay(canInitDelay);

            intakeExtension = new IntakeExtension(config.intakeExtension);
            Timer.delay(canInitDelay);

            fuelIntake = new FuelIntake(config.fuelIntake);
            Timer.delay(canInitDelay);

            turret = new Turret(config.turret);
            Timer.delay(canInitDelay);

            hood = new Hood(config.hood);
            Timer.delay(canInitDelay);

            launcher = new Launcher(config.launcher);
            Timer.delay(canInitDelay);

            launcherTower = new LauncherTower(config.launcherTower);
            Timer.delay(canInitDelay);

            dyeRotor = new DyeRotor(config.dyeRotor);
            Timer.delay(canInitDelay);

            superStructure =
                    new SuperStructure(
                            swerve,
                            fuelIntake,
                            intakeExtension,
                            dyeRotor,
                            launcher,
                            launcherTower,
                            turret,
                            hood);

            auton = new Auton(superStructure);
            vision = new Vision(config.vision);
            batteryLogger = new BatteryLogger();
            // leds = new Leds();

            if (RobotBase.isSimulation()) {
                robotSim = new RobotSim(superStructure);
            }

            configureBindings();

            batteryLogger.setEnabled(true);

            Telemetry.print("--- Robot Init Complete ---");

        } catch (Throwable t) {
            // intercept error and log it
            CrashTracker.logThrowableCrash(t);
            throw t;
        }

        RobotController.setBrownoutVoltage(Units.Volts.of(4.6));

        // Logged once; the robot app reads these over NetworkTables, so publish them directly.
        Telemetry.logDashAlways("BuildConstants/ProjectName", BuildConstants.MAVEN_NAME);
        Telemetry.logDashAlways("BuildConstants/BuildDate", BuildConstants.BUILD_DATE);
        Telemetry.logDashAlways("BuildConstants/GitSHA", BuildConstants.GIT_SHA);
        Telemetry.logDashAlways("BuildConstants/GitDate", BuildConstants.GIT_DATE);
        Telemetry.logDashAlways("BuildConstants/GitBranch", BuildConstants.GIT_BRANCH);
        Telemetry.logDashAlways(
                "BuildConstants/GitDirty",
                switch (BuildConstants.DIRTY) {
                    case 0 -> "All changes committed";
                    case 1 -> "Uncommitted changes";
                    default -> "Unknown";
                });
    }
    /** Configures the bindings. */
    public void configureBindings() {
        // LT alone → intake fuel; do nothing if RT is already held (RT+LT handled below)
        pilot.LT.onTrue(
                Commands.either(
                        superStructure.setStateCommand(WantedSuperState.INTAKE_FUEL),
                        Commands.none(),
                        pilot.RT.negate()));

        // RT alone → launch; do nothing if LT is already held (RT+LT handled below)
        pilot.RT
                .onTrue(
                        Commands.either(
                                superStructure.setStateCommand(
                                        WantedSuperState.LAUNCH_WITH_SQUEEZE),
                                Commands.none(),
                                pilot.LT.negate()))
                .onFalse(FeedTargetFactory.feedDefault());
        // RT + LT both held → launch (intake stays extended; resolves to LAUNCH_WITHOUT_SQUEEZE)
        pilot.RT
                .and(pilot.LT)
                .onTrue(superStructure.setStateCommand(WantedSuperState.LAUNCH_WITHOUT_SQUEEZE))
                .onFalse(FeedTargetFactory.feedDefault());

        // LT released while RT still held → launch (no delay; resolves to
        // LAUNCH_WITH_SQUEEZE_WITH_NO_DELAY)
        pilot.LT.onFalse(
                Commands.either(
                        superStructure.setStateCommand(
                                WantedSuperState.LAUNCH_WITH_SQUEEZE_WITH_NO_DELAY),
                        Commands.none(),
                        pilot.RT));

        pilot.RT
                .and(pilot.LB)
                .onTrue(superStructure.setStateCommand(WantedSuperState.LAUNCH_WITH_BRAKE));
        pilot.RT.and(pilot.LB).onFalse(superStructure.setStateCommand(WantedSuperState.IDLE));

        // RT released while LT still held → resume intaking
        pilot.RT.onFalse(
                Commands.either(
                        superStructure.setStateCommand(WantedSuperState.INTAKE_FUEL),
                        Commands.none(),
                        pilot.LT));

        // Both released → idle
        pilot.RT.or(pilot.LT).onFalse(superStructure.setStateCommand(WantedSuperState.IDLE));

        pilot.XButton.whileTrue(superStructure.setStateCommand(WantedSuperState.TRACK_TARGET));
        pilot.XButton.onFalse(
                Commands.either(
                        Commands.none(),
                        superStructure.setStateCommand(WantedSuperState.IDLE),
                        superStructure::currentStateIsLaunching));

        pilot.AButton.whileTrue(superStructure.setStateCommand(WantedSuperState.UNJAM));
        pilot.AButton.onFalse(superStructure.setStateCommand(WantedSuperState.IDLE));

        pilot.home_select.onTrue(superStructure.setStateCommand(WantedSuperState.FORCE_HOME));
        pilot.home_select.onFalse(superStructure.setStateCommand(WantedSuperState.IDLE));

        operator.dPadDown.onTrue(ShotCalculator.decreaseHoodAngleOffset());
        operator.dPadUp.onTrue(ShotCalculator.increaseHoodAngleOffset());
        operator.dPadRight.onTrue(ShotCalculator.increaseTurretAngleOffset());
        operator.dPadLeft.onTrue(ShotCalculator.decreaseTurretAngleOffset());

        // Held: feed regardless of the shot-readiness gates, for a bad sensor or a deliberate dump.
        superStructure.setFeedOverride(operator.YButton);

        operator.LB.onTrue(FeedTargetFactory.feedLeft());
        operator.RB.onTrue(FeedTargetFactory.feedRight());
        operator.LB.or(operator.RB).onFalse(FeedTargetFactory.feedDefault());

        pilot.upReorient.onTrue(swerve.reorientForward());
        pilot.leftReorient.onTrue(swerve.reorientLeft());
        pilot.downReorient.onTrue(swerve.reorientBack());
        pilot.rightReorient.onTrue(swerve.reorientRight());

        pilot.upReorient
                .or(pilot.leftReorient)
                .or(pilot.downReorient)
                .or(pilot.rightReorient)
                .onTrue(pilot.rumbleCommand(1, 0.5).withName("Pilot.reorientRumble"));

        pilot.coastA.onTrue(
                intakeExtension.coastModeCommand().alongWith(turret.coastModeCommand()));
        pilot.brakeB.onTrue(intakeExtension.brakeModeCommand());
        pilot.visionPoseReset_LB_Select.onTrue(vision.resetVisionPoseCommand());

        operator.coastA.onTrue(
                intakeExtension.coastModeCommand().andThen(turret.coastModeCommand()));
        // Point the turret away from the intake by hand, then press B while disabled.
        operator.zeroTurretB.onTrue(turret.zeroTurretCommand());

        Util.autoMode.onTrue(Commands.runOnce(ShiftHelpers::initialize));
        Util.disabled.onTrue(Commands.runOnce(ShiftHelpers::initialize).ignoringDisable(true));

        // Auton Triggers
        Auton.autonIntake.onTrue(
                superStructure.setStateCommand(WantedSuperState.AUTON_INTAKE_FUEL));
        Auton.autonShotPrep.onTrue(
                superStructure.setStateCommand(WantedSuperState.AUTON_TRACK_TARGET));
        Auton.autonShoot.onTrue(
                superStructure.setStateCommand(WantedSuperState.AUTON_LAUNCH_WITH_SQUEEZE));
        Auton.autonShootWithIntake.onTrue(
                superStructure.setStateCommand(WantedSuperState.AUTON_LAUNCH_WITHOUT_SQUEEZE));
        Auton.autonUnjam.onTrue(
                Commands.sequence(
                        superStructure.setStateCommand(WantedSuperState.UNJAM),
                        Commands.waitSeconds(1),
                        superStructure.setStateCommand(WantedSuperState.LAUNCH_WITH_SQUEEZE)));
        Auton.autonClearState.onTrue(superStructure.setStateCommand(WantedSuperState.IDLE));
    }
    /** Configures the sim bindings. */
    public void configureSimBindings() {
        Trigger simLaunching = new Trigger(superStructure::currentStateIsLaunching);
        simLaunching.whileTrue(robotSim.ballSimLaunchFuel());

        // Sim bindings for when people with just keyboards at home are doing sim at home
        pilot.YButton.whileTrue(
                superStructure.setStateCommand(WantedSuperState.LAUNCH_WITH_SQUEEZE));
        pilot.YButton.onFalse(superStructure.setStateCommand(WantedSuperState.IDLE));
        pilot.BButton.whileTrue(superStructure.setStateCommand(WantedSuperState.INTAKE_FUEL));
        pilot.BButton.onFalse(superStructure.setStateCommand(WantedSuperState.IDLE));
        pilot.LB.onTrue(FeedTargetFactory.feedLeft());
        pilot.RB.onTrue(FeedTargetFactory.feedRight());
    }

    /** Sets up the SmartDashboard data for visualization. */
    public void setupSmartDashboardData() {
        SmartDashboard.putData("Field2d", field2d);
    }
    /** Robot init. */
    @Override
    public void robotInit() {
        setupSmartDashboardData();
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    }

    /* ROBOT PERIODIC  */
    /**
     * This method is called periodically the entire time the robot is running. Periodic methods are
     * called every 20 ms (50 times per second) by default Since the robot software is always
     * looping you shouldn't pause the execution of the robot code This ensures that new values are
     * updated from the gamepads and sent to the motors
     */
    @Override
    public void robotPeriodic() {
        RobotLoop.next();
        systemLoad.periodic();
        /*
         * Real-time priority for the loop body only. On 2026-09-05 the rio CPU sat at 92-95% and
         * the DogLog, NetworkTables and JIT threads preempted this thread in the middle of loops;
         * every section of the loop stretched together, which is what preemption looks like. The
         * finally block hands the CPU back so those threads get the rest of the period.
         */
        Threads.setCurrentThreadPriority(true, 99);
        try {
            Telemetry.time("Scheduler/robotPeriodic");

            // Start every loop with an empty shot-solution cache so the first mechanism to ask
            // computes it from this loop's pose.
            ShotCalculator.getInstance().clearShootingParameters();

            // Vision first: this loop's pose correction lands before any mechanism computes a
            // shot. Vision is intentionally not registered with the scheduler.
            Telemetry.time("Scheduler/Vision");
            vision.periodic();
            Telemetry.timeEnd("Scheduler/Vision");

            // SuperStructure second: state decisions reach the mechanism periodics in this same
            // loop rather than the next one. SuperStructure is not a Subsystem.
            Telemetry.time("Scheduler/SuperStructure");
            superStructure.periodic();
            Telemetry.timeEnd("Scheduler/SuperStructure");

            /*
             * Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled
             * commands, running already-scheduled commands, removing finished or interrupted
             * commands, and running subsystem periodic() methods. This must be called from the
             * robot's periodic block in order for anything in the Command-based framework to work.
             */
            Telemetry.time("Scheduler/CommandScheduler");
            CommandScheduler.getInstance().run();
            Telemetry.timeEnd("Scheduler/CommandScheduler");

            Telemetry.log("Match Data/MatchTime", DriverStation.getMatchTime(), "seconds");
            var shift = ShiftHelpers.getOfficialShiftInfo();
            Telemetry.log("Match Data/InShift", shift.active());
            Telemetry.logDash("Match Data/TimeLeftInShift", shift.remainingTime(), "seconds");

            batteryLogger.setBatteryVoltage(RobotController.getBatteryVoltage());
            batteryLogger.setRioCurrent(RobotController.getInputCurrent());
            batteryLogger.logPower();

            logCanBusStatus();

            field2d.setRobotPose(swerve.getRobotPose());

            Telemetry.timeEnd("Scheduler/robotPeriodic");
        } catch (Throwable t) {
            // intercept error and log it
            CrashTracker.logThrowableCrash(t);
            throw t;
        } finally {
            Threads.setCurrentThreadPriority(false, 10);
        }
    }

    /** FPGA time of the last CANivore status read. */
    private double lastCanStatusSeconds = Double.NEGATIVE_INFINITY;

    /**
     * Reads and logs CANivore bus health once a second.
     *
     * <p>CTRE documents {@code CANBus.getStatus()} as blocking for up to 1 ms, and it ran every
     * loop through 2026-09-05: up to 5% of the budget for counters that are cumulative and a
     * utilization figure that moves over seconds. Nothing is lost at 1 Hz; a bus-off or a TX-full
     * event still shows within a second.
     */
    private void logCanBusStatus() {
        double now = Timer.getFPGATimestamp();
        if (now - lastCanStatusSeconds < 1.0) {
            return;
        }
        lastCanStatusSeconds = now;

        var canInfo = mainCANBus.getStatus();
        Telemetry.logDashAlways("CANivore/BusUtilization", canInfo.BusUtilization * 100, "%");
        Telemetry.log("CANivore/BusOffCount", canInfo.BusOffCount);
        Telemetry.log("CANivore/TxFullCount", canInfo.TxFullCount);
        Telemetry.log("CANivore/ReceiveErrorCounter", canInfo.REC);
        Telemetry.logDashAlways("CANivore/TransmitErrorCounter", canInfo.TEC);
    }

    /** Seconds between deliberate full collections while sitting disabled. */
    private static final double DISABLED_GC_PERIOD_SECONDS = 60.0;

    private double lastDisabledGcSeconds = Double.NEGATIVE_INFINITY;

    /**
     * Runs a full garbage collection now, while the robot cannot move.
     *
     * <p>Serial GC's one long pause is the full collection it runs when old gen fills; on a 100 MB
     * heap on this CPU that is the half-second-and-up class of stall, and while it runs the loop
     * stops feeding the watchdog and the robot drops out mid-match. Emptying old gen at every
     * disable (which includes the auto-to-teleop gap) and once a minute while sitting disabled
     * means every enabled period starts with as much headroom as the heap has. The pause still
     * happens; it happens here, where it costs nothing.
     */
    private void collectGarbageWhileDisabled() {
        lastDisabledGcSeconds = Timer.getFPGATimestamp();
        System.gc();
    }

    /** Disabled init. */
    @Override
    public void disabledInit() {
        Telemetry.print("### Disabled Init Starting ### ");
        collectGarbageWhileDisabled();

        // Put the robot back on the selected auto's starting pose. On the field vision overwrites
        // this within a loop or two; in simulation it is the only thing that ever does it.
        placeAtAutoStart = true;

        if (!autonWarmedUp) {
            Command autonStartCommand =
                    Commands.sequence(
                                    FollowPathCommand.warmupCommand(),
                                    PathfindingCommand.warmupCommand(),
                                    Commands.runOnce(
                                            () -> {
                                                Telemetry.log("Initialized", true);
                                                autonWarmedUp = true;
                                            }))
                            .ignoringDisable(true);
            CommandScheduler.getInstance().schedule(autonStartCommand);
        }

        Telemetry.print("### Disabled Init Complete ### ");
    }

    String autoName = "";

    /** Paths of the currently selected auto, kept so the start pose can be re-applied. */
    private List<PathPlannerPath> selectedAutoPaths = new ArrayList<>();

    /**
     * Set whenever the robot should be put back on the selected auto's starting pose.
     *
     * <p>The reset used to happen only when the chooser selection changed, which is fine once and
     * wrong every time after. Nothing else places the robot: all three autos in the chooser carry
     * {@code resetOdom: false}, and PathPlannerAuto only prepends AutoBuilder.resetOdom when that
     * flag is set, so a run leaves the robot wherever the path ended. On the real field vision
     * seeds the pose back; in simulation there is no vision, so the second run of an auto started
     * from the end of the first.
     */
    private boolean placeAtAutoStart = true;

    /** Disabled periodic. */
    @Override
    public void disabledPeriodic() {
        if (Timer.getFPGATimestamp() - lastDisabledGcSeconds >= DISABLED_GC_PERIOD_SECONDS) {
            collectGarbageWhileDisabled();
        }

        String fullAutoName = auton.getAutonomousCommand().getName();
        boolean leftStart = !fullAutoName.endsWith(" - Right");
        List<PathPlannerPath> pathPlannerPaths = new ArrayList<>();

        /*
         * The alliance belongs in the reload key, not just the auto name.
         *
         * The red flip is applied inside the reload branch below, so keying on the name alone meant
         * picking an auto and then setting the alliance never re-flipped anything: the path became
         * red-side while the pose stayed where it was placed under blue, and the robot set off
         * across the field to reach its own start point. In simulation that is the normal order of
         * operations -- the sim DS reports no alliance until you set one.
         */
        String selectionKey =
                fullAutoName + "|" + DriverStation.getAlliance().map(Enum::name).orElse("NONE");

        if (fullAutoName.equals("Do Nothing")) {
            field2d.getObject("Auto Routine").setPoses(new ArrayList<>());
            autoName = selectionKey;
            selectedAutoPaths = new ArrayList<>();
            return;
        }

        // Strip " - Left" / " - Right" suffix to get the base path name
        String baseAutoName = fullAutoName;
        if (baseAutoName.endsWith(" - Left") || baseAutoName.endsWith(" - Right")) {
            baseAutoName = baseAutoName.substring(0, baseAutoName.lastIndexOf(" - "));
        }

        // Reload on an auto switch, a side switch, or an alliance change — each one changes the
        // trajectory that gets flown and therefore where the robot has to be sitting.
        if (!autoName.equals(selectionKey)) {
            autoName = selectionKey;
            Telemetry.log("Auton Warmed Up", false);
            // Drop the old selection's paths now, so a load failure below cannot leave the robot
            // being placed on the previous auto's start pose.
            selectedAutoPaths = new ArrayList<>();

            if (AutoBuilder.getAllAutoNames().contains(baseAutoName)) {
                try {
                    pathPlannerPaths = PathPlannerAuto.getPathGroupFromAutoFile(baseAutoName);
                } catch (IOException | ParseException e) {
                    Telemetry.print("Could not load path planner paths");
                }

                // Flip the paths if on red alliance
                Optional<Alliance> alliance = DriverStation.getAlliance();
                if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                    pathPlannerPaths =
                            pathPlannerPaths.stream()
                                    .map(PathPlannerPath::flipPath)
                                    .collect(Collectors.toList());
                }

                // Mirror the paths if starting on the right
                if (!leftStart) {
                    pathPlannerPaths =
                            pathPlannerPaths.stream()
                                    .map(PathPlannerPath::mirrorPath)
                                    .collect(Collectors.toList());
                }

                if (!pathPlannerPaths.isEmpty()) {
                    // Placing the robot happens below, so a re-disable gets it too.
                    selectedAutoPaths = pathPlannerPaths;
                    placeAtAutoStart = true;

                    // Warm up the starting path
                    Command warmUpPath =
                            Commands.sequence(
                                            AutoBuilder.followPath(pathPlannerPaths.get(0))
                                                    .withTimeout(0.5),
                                            Commands.runOnce(
                                                    () -> {
                                                        Telemetry.print(
                                                                "Auton Warmed Up",
                                                                PrintPriority.HIGH);
                                                        Telemetry.log("Auton Warmed Up", true);
                                                    }))
                                    .ignoringDisable(true);
                    CommandScheduler.getInstance().schedule(warmUpPath);
                } else {
                    Telemetry.print("Warning: No paths loaded for auto: " + baseAutoName);
                }

                // Convert path points to poses
                List<Pose2d> poses = new ArrayList<>();
                for (PathPlannerPath path : pathPlannerPaths) {
                    poses.addAll(
                            path.getAllPathPoints().stream()
                                    .map(
                                            point ->
                                                    new Pose2d(
                                                            point.position.getX(),
                                                            point.position.getY(),
                                                            Rotation2d.kZero))
                                    .collect(Collectors.toList()));
                }
                field2d.getObject("Auto Routine").setPoses(poses);
            } else {
                field2d.getObject("Auto Routine").setPoses(new ArrayList<>());
            }
        }

        // Outside the selection-changed branch on purpose: this also has to run after a disable,
        // when the name has not changed but the robot is sitting wherever the last run left it.
        if (placeAtAutoStart && !selectedAutoPaths.isEmpty()) {
            swerve.resetPose(
                    selectedAutoPaths.get(0).getStartingHolonomicPose().orElse(new Pose2d()));
            placeAtAutoStart = false;
        }
    }
    /** Disabled exit. */
    @Override
    public void disabledExit() {
        Telemetry.print("### Disabled Exit### ");
    }

    /* AUTONOMOUS MODE (AUTO) */
    /**
     * This mode is run when the DriverStation Software is set to autonomous and enabled. In this
     * mode the robot is not able to read values from the gamepads
     */

    /** This method is called once when autonomous starts */
    @Override
    public void autonomousInit() {
        Telemetry.print("@@@ Auton Init @@@ ");

        if (Utils.isSimulation()) {
            robotSim.getBallSim().clearBalls();
            robotSim.getBallSim().placeFieldBalls();
        }

        try {
            auton.init();
        } catch (Throwable t) {
            // intercept error and log it
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    /** Autonomous periodic. */
    @Override
    public void autonomousPeriodic() {}

    /** Autonomous exit. */
    @Override
    public void autonomousExit() {
        auton.exit();
        CommandScheduler.getInstance().cancelAll();
        superStructure.setWantedSuperState(WantedSuperState.IDLE);
        Telemetry.print("@@@ Auton Exit @@@ ");
    }
    /** Teleop init. */
    @Override
    public void teleopInit() {
        try {
            Telemetry.print("!!! Teleop Init Starting !!! ");

            CommandScheduler.getInstance().cancelAll();
            superStructure.setWantedSuperState(WantedSuperState.IDLE);
            field2d.getObject("Auto Routine").setPoses(new ArrayList<>()); // clears auto visualizer

            Telemetry.print("!!! Teleop Init Complete !!! ");
        } catch (Throwable t) {
            // intercept error and log it
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }
    /** Teleop periodic. */
    @Override
    public void teleopPeriodic() {}
    /** Teleop exit. */
    @Override
    public void teleopExit() {
        if (DriverStation.isFMSAttached()) {
            vision.triggerRewindCaptureForAllCameras();
        }
        Telemetry.print("!!! Teleop Exit !!! ");
    }

    /* TEST MODE */
    /**
     * This mode is run when the DriverStation Software is set to test and enabled. In this mode the
     * is fully enabled and can move it's outputs and read values from the gamepads. This mode is
     * never enabled by the competition field It can be used to test specific features or modes of
     * the robot
     */

    /** This method is called once when test mode starts */
    @Override
    public void testInit() {
        try {

            Telemetry.print("~~~ Test Init Starting ~~~ ");

            Telemetry.print("~~~ Test Init Complete ~~~ ");
        } catch (Throwable t) {
            // intercept error and log it
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }
    /** Test periodic. */
    @Override
    public void testPeriodic() {}
    /** Test exit. */
    @Override
    public void testExit() {
        Telemetry.print("~~~ Test Exit ~~~ ");
    }

    /* SIMULATION MODE */
    /**
     * This mode is run when the software is running in simulation and not on an actual robot. This
     * mode is never enabled by the competition field
     */

    /** This method is called once when a simulation starts */
    @Override
    public void simulationInit() {
        Telemetry.print("$$$ Simulation Init Starting $$$ ");
        configureSimBindings();
        Telemetry.print("$$$ Simulation Init Complete $$$ ");
    }

    /** This method is called periodically during simulation. */
    @Override
    public void simulationPeriodic() {
        robotSim.getBallSim().tick(); // runs physics, publishes ball positions to NT
        robotSim.updateArticulatedMechanisms();
        Telemetry.log("Sim/Fuel", robotSim.getBallSim().getTotalIntaked());
    }
}
