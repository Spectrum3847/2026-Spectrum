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
import edu.wpi.first.wpilibj.RobotBase;
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
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConfig;
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
 * Robot entry point. Picks the config for the Rio serial it is running on, then builds the gamepads
 * and mechanisms.
 */
public class Robot extends SpectrumRobot {
    @Getter private static RobotSim robotSim;
    @Getter private static Config config;
    @Getter private static final Field2d field2d = new Field2d();
    public static Telemetry telemetry = new Telemetry();
    public static boolean autonWarmedUp = false;

    public static class Config {
        public final SwerveConfig swerve = new SwerveConfig();
        public final PilotConfig pilot = new PilotConfig();
        public final OperatorConfig operator = new OperatorConfig();
        public final FuelIntakeConfig fuelIntake = new FuelIntakeConfig();
        public final IntakeExtensionConfig intakeExtension = new IntakeExtensionConfig();
        public final IndexerTowerConfig indexerTower = new IndexerTowerConfig();
        public final IndexerBedConfig indexerBed = new IndexerBedConfig();
        public final LauncherConfig launcher = new LauncherConfig();
        public final HoodConfig hood = new HoodConfig();
        public final VisionConfig vision = new VisionConfig();
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
    @Getter private static Leds leds;
    @Getter private static Auton auton;
    @Getter private static SuperStructure superStructure;
    @Getter private static BatteryLogger batteryLogger;
    @Getter private static CANBus mainCANBus;
    private final SystemLoadMonitor systemLoad = new SystemLoadMonitor();

    public Robot() {
        super();
        Telemetry.start(
                RobotBase.isSimulation(), true, false, true, false, true, PrintPriority.NORMAL);

        try {
            Telemetry.print("--- Robot Init Starting ---");

            switch (Rio.id) {
                case PHOTON2026:
                    config = new PHOTON2026();
                    break;
                case PM_2026:
                    config = new PM2026();
                    break;
                default: // SIM and UNKNOWN
                    config = new FM2026();
                    break;
            }

            double canInitDelay = 0.1; // separates the mechanisms' CAN config writes
            mainCANBus = new CANBus(Rio.CANIVORE); // "*" takes the first CANivore found

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

            auton = new Auton(superStructure);
            vision = new Vision(config.vision);
            batteryLogger = new BatteryLogger();

            if (Utils.isSimulation()) {
                robotSim = new RobotSim(superStructure);
                configureSimBindings();
            }

            configureBindings();

            batteryLogger.setEnabled(true);

            Telemetry.print("--- Robot Init Complete ---");

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }

        RobotController.setBrownoutVoltage(Units.Volts.of(4.6));

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

    public void configureBindings() {
        // LT alone intakes. RT wins, so the guard yields when RT is held.
        pilot.LT.onTrue(
                Commands.either(
                        superStructure.setStateCommand(WantedSuperState.INTAKE_FUEL),
                        Commands.none(),
                        pilot.RT.negate()));

        // RT alone launches with a squeeze. LT yields, so RT+LT falls through to the binding below.
        pilot.RT.onTrue(
                Commands.either(
                        superStructure.setStateCommand(WantedSuperState.LAUNCH_WITH_SQUEEZE),
                        Commands.none(),
                        pilot.LT.negate()));

        // RT+LT together launches with the intake held out.
        pilot.RT
                .and(pilot.LT)
                .onTrue(superStructure.setStateCommand(WantedSuperState.LAUNCH_WITHOUT_SQUEEZE));

        // LT up while RT is held launches with no squeeze delay.
        pilot.LT.onFalse(
                Commands.either(
                        superStructure.setStateCommand(
                                WantedSuperState.LAUNCH_WITH_SQUEEZE_WITH_NO_DELAY),
                        Commands.none(),
                        pilot.RT));

        // RT up while LT is held resumes the intake.
        pilot.RT.onFalse(
                Commands.either(
                        superStructure.setStateCommand(WantedSuperState.INTAKE_FUEL),
                        Commands.none(),
                        pilot.LT));

        // Both triggers up goes idle.
        pilot.RT.or(pilot.LT).onFalse(superStructure.setStateCommand(WantedSuperState.IDLE));

        pilot.LT.and(pilot.LB).onTrue(superStructure.setStateCommand(WantedSuperState.EJECT));
        pilot.LT.and(pilot.LB).onFalse(superStructure.setStateCommand(WantedSuperState.IDLE));

        pilot.XButton.onTrue(superStructure.setStateCommand(WantedSuperState.TRACK_TARGET));
        pilot.XButton.onFalse(superStructure.setStateCommand(WantedSuperState.IDLE));

        pilot.AButton.onTrue(superStructure.setStateCommand(WantedSuperState.UNJAM));
        pilot.AButton.onFalse(superStructure.setStateCommand(WantedSuperState.IDLE));

        pilot.selectButton.onTrue(superStructure.setStateCommand(WantedSuperState.FORCE_HOME));
        pilot.selectButton.onFalse(superStructure.setStateCommand(WantedSuperState.IDLE));

        pilot.dPadUp.and(pilot.LB).onTrue(swerve.reorientForward());
        pilot.dPadLeft.and(pilot.LB).onTrue(swerve.reorientLeft());
        pilot.dPadDown.and(pilot.LB).onTrue(swerve.reorientBack());
        pilot.dPadRight.and(pilot.LB).onTrue(swerve.reorientRight());

        Util.disabled.and(pilot.AButton).onTrue(superStructure.coastMechanisms());
        Util.disabled.and(pilot.BButton).onTrue(superStructure.brakeMechanisms());

        Util.disabled.and(operator.AButton).onTrue(superStructure.coastMechanisms());
        Util.disabled.and(operator.BButton).onTrue(superStructure.brakeMechanisms());

        operator.LB
                .and(operator.YButton)
                .onTrue(
                        Commands.parallel(
                                        intakeExtension.resetCurrentPositionToMaxCommand(),
                                        operator.rumbleCommand(1, 0.5))
                                .ignoringDisable(true));

        operator.selectButton.onTrue(superStructure.setStateCommand(WantedSuperState.FORCE_HOME));
        operator.selectButton.onFalse(superStructure.setStateCommand(WantedSuperState.IDLE));

        operator.dPadDown.onTrue(ShotCalculator.decreaseHoodAngleOffset());
        operator.dPadUp.onTrue(ShotCalculator.increaseHoodAngleOffset());
        operator.dPadRight.onTrue(ShotCalculator.decreaseDriveAngleOffset());
        operator.dPadLeft.onTrue(ShotCalculator.increaseDriveAngleOffset());

        // Reset the shift timer whenever the robot enables.
        Util.teleop.onTrue(Commands.runOnce(ShiftHelpers::initialize));
        Util.autoMode.onTrue(Commands.runOnce(ShiftHelpers::initialize));
        Util.disabled.onTrue(Commands.runOnce(ShiftHelpers::initialize).ignoringDisable(true));

        Auton.autonIntake.onTrue(
                superStructure.setStateCommand(WantedSuperState.AUTON_INTAKE_FUEL));
        Auton.autonShotPrep.onTrue(
                superStructure.setStateCommand(WantedSuperState.AUTON_TRACK_TARGET));
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

    public void setupSmartDashboardData() {
        SmartDashboard.putData("Field2d", field2d);
    }

    @Override
    public void robotInit() {
        setupSmartDashboardData();

        // Build the ShotCalculator now so its Hub Model Chooser is on the dashboard before
        // enabling.
        ShotCalculator.getInstance();

        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    }

    /**
     * Runs the command scheduler. WPILib calls this every 20 ms, and nothing in the command
     * framework moves without it.
     */
    @Override
    public void robotPeriodic() {
        RobotLoop.next();
        systemLoad.periodic();
        try {
            Telemetry.time("Scheduler/robotPeriodic");
            CommandScheduler.getInstance().run();

            Telemetry.logDash("Match Data/MatchTime", DriverStation.getMatchTime(), "seconds");
            Telemetry.logDash("Match Data/InShift", ShiftHelpers.getOfficialShiftInfo().active());
            Telemetry.logDash(
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
                                                Telemetry.logDashAlways("Initialized", true);
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

        // The field visualizer keys off the side suffix, so strip it to get the base path name
        String baseAutoName = fullAutoName;
        if (baseAutoName.endsWith(" - Left") || baseAutoName.endsWith(" - Right")) {
            baseAutoName = baseAutoName.substring(0, baseAutoName.lastIndexOf(" - "));
        }

        // Reload on any name change, whether the auto or the side switched.
        if (!autoName.equals(fullAutoName)) {
            autoName = fullAutoName;
            Telemetry.log("Auton Warmed Up", false);

            if (AutoBuilder.getAllAutoNames().contains(baseAutoName)) {
                try {
                    pathPlannerPaths = PathPlannerAuto.getPathGroupFromAutoFile(baseAutoName);
                } catch (IOException | ParseException e) {
                    Telemetry.print("Could not load path planner paths");
                }

                Optional<Alliance> alliance = DriverStation.getAlliance();
                if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                    pathPlannerPaths =
                            pathPlannerPaths.stream()
                                    .map(PathPlannerPath::flipPath)
                                    .collect(Collectors.toList());
                }

                if (!leftStart) {
                    pathPlannerPaths =
                            pathPlannerPaths.stream()
                                    .map(PathPlannerPath::mirrorPath)
                                    .collect(Collectors.toList());
                }

                if (!pathPlannerPaths.isEmpty()) {
                    swerve.resetPose(
                            pathPlannerPaths
                                    .get(0)
                                    .getStartingHolonomicPose()
                                    .orElse(new Pose2d()));

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

            superStructure.setWantedSuperState(WantedSuperState.IDLE);
            field2d.getObject("Auto Routine").setPoses(new ArrayList<>()); // clears the visualizer

            Telemetry.print("!!! Teleop Init Complete !!! ");
        } catch (Throwable t) {
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

    @Override
    public void testInit() {
        try {

            Telemetry.print("~~~ Test Init Starting ~~~ ");

            Telemetry.print("~~~ Test Init Complete ~~~ ");
        } catch (Throwable t) {
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

    @Override
    public void simulationInit() {
        Telemetry.print("$$$ Simulation Init Starting $$$ ");
        Telemetry.print("$$$ Simulation Init Complete $$$ ");
    }

    @Override
    public void simulationPeriodic() {
        robotSim.getBallSim().tick(); // ticks the physics and publishes ball positions to NT
        robotSim.updateArticulatedMechanisms();
        Telemetry.log("Sim/Fuel", robotSim.getBallSim().getTotalIntaked());
    }
}
