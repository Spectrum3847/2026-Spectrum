package frc.robot;

import com.ctre.phoenix6.CANBus;
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
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.rebuilt.ShiftHelpers;
import frc.rebuilt.ShotCalculator;
import frc.robot.auton.Auton;
import frc.robot.configs.FM2026;
import frc.robot.configs.PHOTON2026;
import frc.robot.configs.PM2026;
import frc.robot.operator.Operator;
import frc.robot.operator.Operator.OperatorConfig;
import frc.robot.pilot.Pilot;
import frc.robot.pilot.Pilot.PilotConfig;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.WantedSuperState;
import frc.robot.subsystems.fuelIntake.FuelIntake;
import frc.robot.subsystems.fuelIntake.FuelIntake.FuelIntakeConfig;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.Hood.HoodConfig;
import frc.robot.subsystems.indexerBed.IndexerBed;
import frc.robot.subsystems.indexerBed.IndexerBed.IndexerBedConfig;
import frc.robot.subsystems.indexerTower.IndexerTower;
import frc.robot.subsystems.indexerTower.IndexerTower.IndexerTowerConfig;
import frc.robot.subsystems.intakeExtension.IntakeExtension;
import frc.robot.subsystems.intakeExtension.IntakeExtension.IntakeExtensionConfig;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.Launcher.LauncherConfig;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConfig;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.Vision.VisionConfig;
import frc.spectrumLib.framework.SpectrumRobot;
import frc.spectrumLib.hardware.Rio;
import frc.spectrumLib.telemetry.BatteryLogger;
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
import org.ironmaple.simulation.SimulatedArena;
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

    public static class Config {
        public SwerveConfig swerve = new SwerveConfig();
        public PilotConfig pilot = new PilotConfig();
        public OperatorConfig operator = new OperatorConfig();
        public FuelIntakeConfig fuelIntake = new FuelIntakeConfig();
        public IntakeExtensionConfig intakeExtension = new IntakeExtensionConfig();
        public IndexerTowerConfig indexerTower = new IndexerTowerConfig();
        public IndexerBedConfig indexerBed = new IndexerBedConfig();
        public LauncherConfig launcher = new LauncherConfig();
        public HoodConfig hood = new HoodConfig();
        public VisionConfig vision = new VisionConfig();
    }

    @Getter private static Swerve swerve;
    @Getter private static FuelIntake fuelIntake;
    @Getter private static IntakeExtension intakeExtension;
    @Getter private static IndexerTower indexerTower;
    @Getter private static IndexerBed indexerBed;
    @Getter private static Operator operator;
    @Getter private static Pilot pilot;
    @Getter private static Launcher launcher;
    @Getter private static Hood hood;
    @Getter private static Vision vision;
    @Getter private static Auton auton;
    @Getter private static SuperStructure superStructure;
    @Getter private static BatteryLogger batteryLogger;
    @Getter private static CANBus mainCANBus;

    public Robot() {
        super();
        Telemetry.start(true, true, false, true, false, true, PrintPriority.NORMAL);

        try {
            Telemetry.print("--- Robot Init Starting ---");

            // Set up the config
            switch (Rio.id) {
                case PHOTON2026:
                    config = new PHOTON2026();
                    break;
                case PM_2026:
                    config = new PM2026();
                    break;
                    // case FM_2026:
                    //     config = new FM2026();
                    //     break;
                default: // SIM and UNKNOWN
                    config = new FM2026();
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

            hood = new Hood(config.hood);
            Timer.delay(canInitDelay);

            launcher = new Launcher(config.launcher);
            Timer.delay(canInitDelay);

            indexerTower = new IndexerTower(config.indexerTower);
            Timer.delay(canInitDelay);

            indexerBed = new IndexerBed(config.indexerBed);
            Timer.delay(canInitDelay);

            superStructure =
                    new SuperStructure(
                            swerve,
                            fuelIntake,
                            intakeExtension,
                            indexerTower,
                            indexerBed,
                            launcher,
                            hood);

            auton = new Auton();
            vision = new Vision(config.vision);
            batteryLogger = new BatteryLogger();

            if (Utils.isSimulation()) {
                robotSim = new RobotSim();
                configureSimBindings();
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

        Telemetry.log("BuildConstants/ProjectName", BuildConstants.MAVEN_NAME);
        Telemetry.log("BuildConstants/BuildDate", BuildConstants.BUILD_DATE);
        Telemetry.log("BuildConstants/GitSHA", BuildConstants.GIT_SHA);
        Telemetry.log("BuildConstants/GitDate", BuildConstants.GIT_DATE);
        Telemetry.log("BuildConstants/GitBranch", BuildConstants.GIT_BRANCH);
        Telemetry.log(
                "BuildConstants/GitDirty",
                switch (BuildConstants.DIRTY) {
                    case 0 -> "All changes committed";
                    case 1 -> "Uncommitted changes";
                    default -> "Unknown";
                });
    }

    public void configureBindings() {
        // LT alone → intake fuel; do nothing if RT is already held (RT+LT handled below)
        pilot.LT.onTrue(
                Commands.either(
                        superStructure.setStateCommand(WantedSuperState.INTAKE_FUEL),
                        Commands.none(),
                        pilot.RT.negate()));

        // RT alone → launch; do nothing if LT is already held (RT+LT handled below)
        pilot.RT.onTrue(
                Commands.either(
                        superStructure.setStateCommand(WantedSuperState.LAUNCH_WITH_SQUEEZE),
                        Commands.none(),
                        pilot.LT.negate()));

        // RT + LT both held → launch (intake stays extended; resolves to LAUNCH_WITHOUT_SQUEEZE)
        pilot.RT
                .and(pilot.LT)
                .onTrue(superStructure.setStateCommand(WantedSuperState.LAUNCH_WITHOUT_SQUEEZE));

        // LT released while RT still held → launch (no delay; resolves to
        // LAUNCH_WITH_SQUEEZE_WITH_NO_DELAY)
        pilot.LT.onFalse(
                Commands.either(
                        superStructure.setStateCommand(
                                WantedSuperState.LAUNCH_WITH_SQUEEZE_WITH_NO_DELAY),
                        Commands.none(),
                        pilot.RT));

        // RT released while LT still held → resume intaking
        pilot.RT.onFalse(
                Commands.either(
                        superStructure.setStateCommand(WantedSuperState.INTAKE_FUEL),
                        Commands.none(),
                        pilot.LT));

        // Both released → idle
        pilot.RT.or(pilot.LT).onFalse(superStructure.setStateCommand(WantedSuperState.IDLE));

        pilot.XButton.whileTrue(superStructure.setStateCommand(WantedSuperState.TRACK_TARGET));
        pilot.XButton.onFalse(superStructure.setStateCommand(WantedSuperState.IDLE));

        pilot.AButton.whileTrue(superStructure.setStateCommand(WantedSuperState.UNJAM));
        pilot.AButton.onFalse(superStructure.setStateCommand(WantedSuperState.IDLE));

        pilot.selectButton
                .and(pilot.LB)
                .onTrue(superStructure.setStateCommand(WantedSuperState.FORCE_HOME));
        pilot.selectButton
                .and(pilot.LB)
                .onFalse(superStructure.setStateCommand(WantedSuperState.IDLE));

        // Reset hub shift timer when enabling
        Util.teleop.onTrue(Commands.runOnce(ShiftHelpers::initialize));
        Util.autoMode.onTrue(Commands.runOnce(ShiftHelpers::initialize));
        Util.disabled.onTrue(Commands.runOnce(ShiftHelpers::initialize).ignoringDisable(true));

        // Auton Triggers
        Auton.autonIntake.onTrue(superStructure.setStateCommand(WantedSuperState.INTAKE_FUEL));
        Auton.autonShotPrep.onTrue(superStructure.setStateCommand(WantedSuperState.TRACK_TARGET));
        Auton.autonUnjam.onTrue(
                Commands.sequence(
                        superStructure.setStateCommand(WantedSuperState.UNJAM),
                        Commands.waitSeconds(1),
                        superStructure.setStateCommand(WantedSuperState.LAUNCH_WITH_SQUEEZE)));
        Auton.autonClearState.onTrue(superStructure.setStateCommand(WantedSuperState.IDLE));
    }

    public void configureSimBindings() {
        RobotSim.simLaunching().whileTrue(robotSim.ballSimLaunchFuel());
    }

    /** Sets up the SmartDashboard data for visualization. */
    public void setupSmartDashboardData() {
        SmartDashboard.putData("Field2d", field2d);
    }

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
        try {
            Telemetry.time("Scheduler/robotPeriodic");
            /*
             * Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled
             * commands, running already-scheduled commands, removing finished or interrupted
             * commands, and running subsystem periodic() methods. This must be called from the
             * robot's periodic block in order for anything in the Command-based framework to work.
             */
            CommandScheduler.getInstance().run();

            Telemetry.log("Match Data/MatchTime", DriverStation.getMatchTime(), "seconds");
            Telemetry.log("Match Data/InShift", ShiftHelpers.getOfficialShiftInfo().active());
            Telemetry.log(
                    "Match Data/TimeLeftInShift",
                    ShiftHelpers.getOfficialShiftInfo().remainingTime(),
                    "seconds");

            batteryLogger.setBatteryVoltage(RobotController.getBatteryVoltage());
            batteryLogger.setRioCurrent(RobotController.getInputCurrent());
            batteryLogger.logPower();

            var canInfo = mainCANBus.getStatus();
            Telemetry.log("CANivore/BusUtilization", canInfo.BusUtilization * 100, "%");
            Telemetry.log("CANivore/BusOffCount", canInfo.BusOffCount);
            Telemetry.log("CANivore/TxFullCount", canInfo.TxFullCount);
            Telemetry.log("CANivore/ReceiveErrorCounter", canInfo.REC);
            Telemetry.log("CANivore/TransmitErrorCounter", canInfo.TEC);

            field2d.setRobotPose(swerve.getRobotPose());

            ShotCalculator.getInstance().clearShootingParameters();
            Telemetry.timeEnd("Scheduler/robotPeriodic");
        } catch (Throwable t) {
            // intercept error and log it
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledInit() {
        Telemetry.print("### Disabled Init Starting ### ");

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

    @Override
    public void disabledPeriodic() {
        String fullAutoName = auton.getAutonomousCommand().getName();
        boolean leftStart = !fullAutoName.endsWith(" - Right");
        List<PathPlannerPath> pathPlannerPaths = new ArrayList<>();

        if (fullAutoName.equals("Do Nothing")) {
            field2d.getObject("Auto Routine").setPoses(new ArrayList<>());
            autoName = fullAutoName;
            return;
        }

        // Strip " - Left" / " - Right" suffix to get the base path name
        String baseAutoName = fullAutoName;
        if (baseAutoName.endsWith(" - Left") || baseAutoName.endsWith(" - Right")) {
            baseAutoName = baseAutoName.substring(0, baseAutoName.lastIndexOf(" - "));
        }

        // Reload whenever the full name changes — catches both auto switches and side switches
        if (!autoName.equals(fullAutoName)) {
            autoName = fullAutoName;
            Telemetry.log("Auton Warmed Up", false);

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
                    // Set the robot pose to the starting pose of the first path
                    swerve.resetPose(
                            pathPlannerPaths
                                    .get(0)
                                    .getStartingHolonomicPose()
                                    .orElse(new Pose2d()));

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
    }

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

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {
        auton.exit();
        Telemetry.print("@@@ Auton Exit @@@ ");
    }

    @Override
    public void teleopInit() {
        try {
            Telemetry.print("!!! Teleop Init Starting !!! ");

            field2d.getObject("Auto Routine").setPoses(new ArrayList<>()); // clears auto visualizer

            Telemetry.print("!!! Teleop Init Complete !!! ");
        } catch (Throwable t) {
            // intercept error and log it
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void teleopPeriodic() {}

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

    @Override
    public void testPeriodic() {}

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
        SimulatedArena.getInstance().resetFieldForAuto();
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
