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
import frc.robot.fuelIntake.FuelIntake;
import frc.robot.fuelIntake.FuelIntake.FuelIntakeConfig;
import frc.robot.hood.Hood;
import frc.robot.hood.Hood.HoodConfig;
import frc.robot.indexerBed.IndexerBed;
import frc.robot.indexerBed.IndexerBed.IndexerBedConfig;
import frc.robot.indexerTower.IndexerTower;
import frc.robot.indexerTower.IndexerTower.IndexerTowerConfig;
import frc.robot.intakeExtension.IntakeExtension;
import frc.robot.intakeExtension.IntakeExtension.IntakeExtensionConfig;
import frc.robot.launcher.Launcher;
import frc.robot.launcher.Launcher.LauncherConfig;
import frc.robot.operator.Operator;
import frc.robot.operator.Operator.OperatorConfig;
import frc.robot.pilot.Pilot;
import frc.robot.pilot.Pilot.PilotConfig;
import frc.robot.swerve.Swerve;
import frc.robot.swerve.SwerveConfig;
import frc.robot.vision.Vision;
import frc.robot.vision.Vision.VisionConfig;
import frc.robot.vision.VisionSystem;
import frc.spectrumLib.BatteryLogger;
import frc.spectrumLib.Rio;
import frc.spectrumLib.SpectrumRobot;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.Telemetry.PrintPriority;
import frc.spectrumLib.util.CrashTracker;
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
    @Getter private static VisionSystem visionSystem;
    @Getter private static Launcher launcher;
    @Getter private static Hood hood;
    @Getter private static Vision vision;
    @Getter private static Auton auton;
    @Getter private static Coordinator coordinator;
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

            /*
             * Initialize the Subsystems of the robot. Subsystems are how we divide up the robot
             * code. Anything with an output that needs to be independently controlled is a
             * subsystem Something that don't have an output are also subsystems.
             */
            double canInitDelay = 0.1; // Delay between any mechanism with motor/can configs
            mainCANBus = new CANBus("*"); // Use the first CANivore bus found

            coordinator = new Coordinator();
            operator = new Operator(config.operator);
            pilot = new Pilot(config.pilot);
            swerve = new Swerve(config.swerve);
            Timer.delay(canInitDelay);
            vision = new Vision(config.vision);
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
            auton = new Auton();
            batteryLogger = new BatteryLogger();

            if (Utils.isSimulation()) {
                robotSim = new RobotSim();
            }

            setupDefaultCommands();
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

    /**
     * This method cancels all commands and returns subsystems to their default commands and the
     * gamepad configs are reset so that new bindings can be assigned based on mode. This method
     * should be called when each mode is initialized.
     *
     * <p><b>Warning:</b> This method will cause a very large loop overrun, as it rebinds all states
     * to their triggers. Be careful when you call this as it will cause delays in the robot code.
     * It is recommended to call this method at the end of disabledInit and teleopInit, as those are
     * the most common places to need to reset commands and bindings.
     */
    public void resetCommandsAndButtons() {
        CommandScheduler.getInstance().cancelAll(); // Disable any currently running commands
        CommandScheduler.getInstance().getActiveButtonLoop().clear();

        // Reset Config for all gamepads and other button bindings
        pilot.resetConfig();
        operator.resetConfig();

        // Bind Triggers for all subsystems
        setupStates();
        RobotStates.setupStates();
    }

    /**
     * This method cancels all commands and returns subsystems to their default commands. This
     * method should be called when each mode is initialized.
     *
     * <p><b>Warning:</b> This method will cause a very large loop overrun, as it rebinds all states
     * to their triggers. Be careful when you call this as it will cause delays in the robot code.
     * It is recommended to call this method at the end of disabledInit and teleopInit, as those are
     * the most common places to need to reset commands and bindings.
     */
    public void clearCommandsAndButtons() {
        CommandScheduler.getInstance().cancelAll(); // Disable any currently running commands
        CommandScheduler.getInstance().getActiveButtonLoop().clear();

        // Bind Triggers for all subsystems
        setupStates();
        RobotStates.setupStates();
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
            Telemetry.log("Applied State", RobotStates.getAppliedState().toString());

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
        clearCommandsAndButtons();
        resetCommandsAndButtons();

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
        String newAutoName;
        boolean leftStart = true;
        List<PathPlannerPath> pathPlannerPaths = new ArrayList<>();
        newAutoName = auton.getAutonomousCommand().getName();
        leftStart = !newAutoName.endsWith(" - Right");

        if (newAutoName.equals("Do Nothing")) {
            field2d.getObject("Auto Routine").setPoses(new ArrayList<>());
            autoName = newAutoName;
            return;
        }

        // Remove " - Left" or " - Right" suffix if present
        if (newAutoName.endsWith(" - Left") || newAutoName.endsWith(" - Right")) {
            newAutoName = newAutoName.substring(0, newAutoName.lastIndexOf(" - "));
        }

        if (!autoName.equals(newAutoName)) {
            autoName = newAutoName;
            Telemetry.log("Auton Warmed Up", false);

            if (AutoBuilder.getAllAutoNames().contains(autoName)) {
                try {
                    pathPlannerPaths = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
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

                // Set the robot pose to the starting pose of the first path
                swerve.resetPose(
                        pathPlannerPaths.get(0).getStartingHolonomicPose().orElse(new Pose2d()));

                // Warm up the starting path
                Command warmUpPath =
                        Commands.sequence(
                                        AutoBuilder.followPath(pathPlannerPaths.get(0))
                                                .withTimeout(0.5),
                                        Commands.runOnce(
                                                () -> {
                                                    Telemetry.print(
                                                            "Auton Warmed Up", PrintPriority.HIGH);
                                                    Telemetry.log("Auton Warmed Up", true);
                                                }))
                                .ignoringDisable(true);
                CommandScheduler.getInstance().schedule(warmUpPath);

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
            SimulatedArena.getInstance().resetFieldForAuto();
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
            RobotStates.clearState();
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
            resetCommandsAndButtons();

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
        SmartDashboard.putNumber(
                "Sim/FuelCount", RobotSim.getIntakeSimulation().getGamePiecesAmount());
    }
}
