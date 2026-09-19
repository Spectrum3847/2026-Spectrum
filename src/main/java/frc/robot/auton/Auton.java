package frc.robot.auton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.WantedSuperState;
import frc.spectrumLib.telemetry.Telemetry;
import frc.spectrumLib.telemetry.Telemetry.PrintPriority;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import org.json.simple.parser.ParseException;

public class Auton {

    public static final EventTrigger autonIntake = new EventTrigger("intake");
    public static final EventTrigger autonShotPrep = new EventTrigger("shotPrep");
    public static final EventTrigger autonShoot = new EventTrigger("shoot");
    public static final EventTrigger autonShootWithIntake = new EventTrigger("shootWithIntake");
    public static final EventTrigger autonClearState = new EventTrigger("clearState");
    public static final EventTrigger autonUnjam = new EventTrigger("unjam");
    public static final EventTrigger autonPoseUpdate = new EventTrigger("poseUpdate");

    private final SendableChooser<Command> pathChooser = new SendableChooser<>();
    private boolean autoMessagePrinted = true;
    private double autonStart = 0;

    /**
     * This method configures the available autonomous routines that can be selected from the
     * SmartDashboard.
     */
    public void setupSelectors() {

        pathChooser.setDefaultOption("Do Nothing", doNothing());

        pathChooser.addOption("Double Swipe Left", OSTBTB(false));
        pathChooser.addOption("Double Swipe Right", OSTBTB(true));
        pathChooser.addOption("Single Swipe with Depot Left", OSRIPPOFF(false));
        pathChooser.addOption("Single Swipe with Depot Right", OSRIPPOFF(true));
        pathChooser.addOption("2nd Double Swipe Left", TWOMANOSTBTB(false));
        pathChooser.addOption("2nd Double Swipe Right", TWOMANOSTBTB(true));
        pathChooser.addOption("Center 1 Swipe Left", OSCENT(false));
        pathChooser.addOption("Center 1 Swipe Right", OSCENT(true));
        pathChooser.addOption("Center to Depot Left", OSCENTOT(false));
        pathChooser.addOption("Center to Depot Right", OSCENTOT(true));
        pathChooser.addOption("Single Swipe with Depot Cutoff Left", OSRIPOFF_CUTOFF(false));
        pathChooser.addOption("Single Swipe with Depot Cutoff Right", OSRIPOFF_CUTOFF(true));

        SmartDashboard.putData("Auto Chooser", pathChooser);
    }

    @SuppressWarnings("unused")
    private SuperStructure robotSuperStructure;

    /**
     * Creates a new Auton instance.
     *
     * @param robotSuperStructure the robotSuperStructure
     */
    public Auton(SuperStructure robotSuperStructure) {
        this.robotSuperStructure = robotSuperStructure;
        setupSelectors(); // runs the command to start the chooser for auto on shuffleboard
        Telemetry.print("Auton Subsystem Initialized");
    }

    /** Init. */
    public void init() {
        Command autonCommand = getAutonomousCommand();

        if (autonCommand != null) {
            CommandScheduler.getInstance().schedule(autonCommand);
            startAutonTimer();
        } else {
            Telemetry.print("No Auton Command Found");
        }
    }

    /** Exit. */
    public void exit() {
        printAutoDuration();
    }

    /** Do nothing. */
    public Command doNothing() {
        return Commands.print("Do Nothing Auto ran").withName("Do Nothing");
    }

    public Command OSTBTB(boolean mirrored) {
        return Commands.sequence(SpectrumAuton("OSTBTB FULL", mirrored))
                // the "- Right" and "- Left" is added to the name of the command so that when the
                // visualizer checks the name of the command it can determine whether the auto is
                // mirrored or not and correctly mirror the poses
                .withName("OSTBTB FULL - " + (mirrored ? "Right" : "Left"));
    }

    public Command OSRIPPOFF(boolean mirrored) {
        return Commands.sequence(SpectrumAuton("OSRIPPOFF FULL", mirrored))
                // the "- Right" and "- Left" is added to the name of the command so that when the
                // visualizer checks the name of the command it can determine whether the auto is
                // mirrored or not and correctly mirror the poses
                .withName("OSRIPPOFF FULL - " + (mirrored ? "Right" : "Left"));
    }

    // Named TWOMANOSTBTB because Java identifiers can't start with a digit; the auto file it loads
    // is "2MANOSTBTB FULL.auto".
    public Command TWOMANOSTBTB(boolean mirrored) {
        return Commands.sequence(
                        Commands.waitSeconds(2), SpectrumAuton("2MANOSTBTB FULL", mirrored))
                .withName("2MANOSTBTB FULL - " + (mirrored ? "Right" : "Left"));
    }

    public Command OSCENT(boolean mirrored) {
        // File is "OSCENT FULL.auto": the rio is case-sensitive, so the name must match exactly.
        // The suffix must be " - Left"/" - Right" with the spaces, or Robot.disabledPeriodic cannot
        // strip it to find the file and place the robot.
        return Commands.sequence(SpectrumAuton("OSCENT FULL", mirrored), launchWithAgitate())
                .withName("OSCENT FULL - " + (mirrored ? "Right" : "Left"));
    }

    public Command OSCENTOT(boolean mirrored) {
        return Commands.sequence(SpectrumAuton("OSCENTOT FULL", mirrored))
                .withName("OSCENTOT FULL - " + (mirrored ? "Right" : "Left"));
    }

    public Command OSRIPOFF_CUTOFF(boolean mirrored) {
        return Commands.sequence(SpectrumAuton("OSRIPOFF CUTOFF", mirrored))
                // the "- Right" and "- Left" is added to the name of the command so that when the
                // visualizer checks the name of the command it can determine whether the auto is
                // mirrored or not and correctly mirror the poses
                .withName("OSRIPOFF CUTOFF - " + (mirrored ? "Right" : "Left"));
    }

    // Allows Robot to continue shooting even after path has been completed--at a stand still
    public Command launchWithAgitate() {
        // Was an InstantCommand that built the state command inside its lambda and dropped it, so
        // it never set the state. Return the command itself and the sequence schedules it.
        return robotSuperStructure.setStateCommand(WantedSuperState.AUTON_LAUNCH_WITH_SQUEEZE);
    }

    /** Auto names the chooser was built with that have no {@code .auto} file on this rio. */
    private static final List<String> missingAutoFiles = new ArrayList<>();

    private static final Alert missingAutoFileAlert = new Alert("", AlertType.kError);

    /**
     * Checks at boot that an auto the chooser offers actually exists in deploy/pathplanner/autos.
     *
     * <p>PathPlanner reports a missing file to the Driver Station once at construction and then
     * runs an empty command, which is how Chezy 2026-09-19 QM4 sat still for auto: the code said
     * "OSCENT Full", the file said "OSCENT FULL.auto", and the rio's filesystem cares about the
     * difference while the Windows sim does not. This makes it an alert that stays up.
     *
     * @param autoName the exact file name without {@code .auto}
     */
    private static void verifyAutoFile(String autoName) {
        if (AutoBuilder.getAllAutoNames().contains(autoName)) {
            return;
        }
        if (!missingAutoFiles.contains(autoName)) {
            missingAutoFiles.add(autoName);
        }
        missingAutoFileAlert.setText(
                "No .auto file on the rio for: "
                        + String.join(", ", missingAutoFiles)
                        + " (names are case-sensitive on the rio). Those autos will do nothing.");
        missingAutoFileAlert.set(true);
        Telemetry.print(
                "!!! No .auto file named '"
                        + autoName
                        + "' in deploy/pathplanner/autos. That auto will do nothing.",
                PrintPriority.HIGH);
    }

    /**
     * Creates the PathPlannerAuto for a routine, built here at boot so its trajectories are
     * generated and cached before the match (PathPlanner's FollowPathCommand generates the ideal
     * trajectory in its constructor and reuses it at start if the robot is still and within 30 deg
     * of the path's starting heading).
     *
     * <p>Until 2026-09-16 this prepended {@code waitSeconds(0.01)}, a leftover from the 2025
     * migration. A wait command finishes on the scheduler loop after the one it started in, so it
     * cost a full loop, 25 to 40 ms at our loop period, of the robot standing still at the start of
     * every auto. Nothing depended on it: the odometry reset a routine may ask for lives inside
     * PathPlannerAuto itself.
     *
     * @param autoName the name of the autonomous routine to execute
     * @param mirrored whether the autonomous routine should be mirrored
     * @return the auto command
     */
    public Command SpectrumAuton(String autoName, boolean mirrored) {
        verifyAutoFile(autoName);
        return new PathPlannerAuto(autoName, mirrored).withName(autoName);
    }
    /** Spectrum auton, cut off after {@code duration} seconds. */
    public Command SpectrumAuton(String autoName, boolean mirrored, double duration) {
        verifyAutoFile(autoName);
        return new PathPlannerAuto(autoName, mirrored).withTimeout(duration).withName(autoName);
    }

    /**
     * Retrieves the autonomous command selected on the shuffleboard.
     *
     * @return the selected autonomous command if one is chosen; otherwise, returns a PrintCommand
     *     indicating that the autonomous command is null.
     */
    public Command getAutonomousCommand() {
        Command auton = pathChooser.getSelected(); // sees what auto is chosen on shuffleboard
        if (auton != null) {
            return auton; // checks to make sure there is an auto and if there is it runs an auto
        } else {
            return new PrintCommand(
                    "*** AUTON COMMAND IS NULL ***"); // runs if there is no auto chosen, which
            // shouldn't happen because of the default
            // auto set to nothing which still runs
            // something
        }
    }

    /** This method is called in AutonInit */
    public void startAutonTimer() {
        autonStart = Timer.getFPGATimestamp();
        autoMessagePrinted = false;
    }

    /** Called at AutonExit and displays the duration of the auton command Based on 6328 code */
    public void printAutoDuration() {
        Command autoCommand = getAutonomousCommand();
        if (autoCommand != null) {
            if (!autoCommand.isScheduled() && !autoMessagePrinted) {
                if (DriverStation.isAutonomousEnabled()) {
                    Telemetry.print(
                            String.format(
                                    "*** Auton finished in %.2f secs ***",
                                    Timer.getFPGATimestamp() - autonStart));
                } else {
                    Telemetry.print(
                            String.format(
                                    "*** Auton CANCELLED in %.2f secs ***",
                                    Timer.getFPGATimestamp() - autonStart));
                }
                autoMessagePrinted = true;
            }
        }
    }
    /** Follow single path. */
    public static Command followSinglePath(String pathName) {
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path;
        try {
            path = PathPlannerPath.fromPathFile(pathName);

            // Create a path following command using AutoBuilder. This will also trigger event
            // markers.
            return AutoBuilder.followPath(path);
        } catch (FileVersionException | IOException | ParseException e) {
            e.printStackTrace();
        }
        return new PrintCommand("ERROR LOADING PATH");
    }
    /** Pathfinding command to pose. */
    public static Command pathfindingCommandToPose(
            double xPos, double yPos, double rotation, double vel, double accel) {
        // Since we are using a holonomic drivetrain, the rotation component of this pose
        // represents the goal holonomic rotation
        Pose2d targetPose = new Pose2d(xPos, yPos, Rotation2d.fromDegrees(rotation));

        // Create the constraints to use while pathfinding
        PathConstraints constraints =
                new PathConstraints(
                        vel, accel, Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand =
                AutoBuilder.pathfindToPoseFlipped(
                        targetPose, constraints, 0.0 // Goal end velocity in meters/sec
                        );

        return pathfindingCommand;
    }
    // Log Command
    /** Log. */
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
